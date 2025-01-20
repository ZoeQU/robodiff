import os
import time
from multiprocessing.managers import SharedMemoryManager
import numpy as np
from threading import Lock
from diffusion_policy.shared_memory.shared_memory_queue import SharedMemoryQueue, Empty
from diffusion_policy.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from diffusion_policy.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from diffusion_policy.real_world.abb import Robot
import threading
from queue import Queue, Empty  # Thread-safe queue for commands
import numpy as np
import scipy.interpolate as si
import scipy.spatial.transform as st
from scipy.spatial.transform import Rotation as R
import multiprocessing as mp
from enum import Enum


# 四元数归一化函数
def normalize_quaternion(quaternion):
    norm = np.linalg.norm(quaternion)
    if norm == 0:
        raise ValueError("Quaternion has zero norm and cannot be normalized!")
    return quaternion / norm


# 替代 PoseTrajectoryInterpolator
class QuaternionPoseInterpolator:
    def __init__(self, times, poses):
        """
        支持四元数格式 [x, y, z, q1, q2, q3, q4] 的插值器。
        :param times: 时间戳数组
        :param poses: 姿态数组 [x, y, z, q1, q2, q3, q4]
        """
        assert len(times) >= 1
        assert len(times) == len(poses)

        self.times = np.array(times)
        self.positions = np.array([pose[:3] for pose in poses])  # 提取位置部分
        self.rotations = st.Rotation.from_quat([pose[3:] for pose in poses])  # 提取四元数部分并转换为 Rotation 对象

        if len(times) > 1:
            self.pos_interp = si.interp1d(self.times, self.positions, axis=0, assume_sorted=True)
            self.rot_interp = st.Slerp(self.times, self.rotations)
        else:
            self.pos_interp = None  # 单点插值
            self.rot_interp = None

    def __call__(self, t):
        """
        根据时间戳 t 返回插值后的姿态。
        :param t: 单个时间戳或时间戳数组
        :return: 插值后的姿态数组 [x, y, z, q1, q2, q3, q4]
        """
        if self.pos_interp is None or self.rot_interp is None:
            # 单点插值，直接返回初始姿态
            position = self.positions[0]
            quaternion = self.rotations[0].as_quat()
        else:
            t = np.clip(t, self.times[0], self.times[-1])  # 限制时间范围
            position = self.pos_interp(t)
            quaternion = self.rot_interp(t).as_quat()

        return np.hstack((position, quaternion))

class FakeQuaternionPoseInterpolator:
    def __init__(self, times, poses):
        """
        假的插值器，直接返回输入的姿态，不进行任何平滑插值。
        :param times: 时间戳数组
        :param poses: 姿态数组 [x, y, z, q1, q2, q3, q4]
        """
        assert len(times) >= 1
        assert len(times) == len(poses)

        self.times = np.array(times)
        self.poses = np.array(poses)  # 保存原始姿态

    def __call__(self, t):
        """
        根据时间戳 t 返回姿态，直接复制输入的姿态。
        :param t: 单个时间戳或时间戳数组
        :return: 姿态数组 [x, y, z, q1, q2, q3, q4]
        """
        # 如果 t 是单个时间戳，则返回第一个姿态
        if np.isscalar(t):
            return self.poses[0]
        
        # 如果 t 是时间戳数组，则复制第一个姿态 len(t) 次
        t = np.asarray(t)
        return np.tile(self.poses[0], (len(t), 1))
    

# Command 枚举类
class Command(Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

# ABBInterpolationController
class ABBInterpolationController(mp.Process):
    """
    ABB robot controller using interpolation. Runs in a separate process to ensure predictable latency.
    """

    def __init__(self,
                 shm_manager,
                 robot_ip,
                 frequency=125,
                 lookahead_time=0.1,
                 gain=300,
                 max_pos_speed=0.5,  # mm/s
                 max_rot_speed=30,  # deg/s
                 launch_timeout=3,
                 tcp_offset_pose=None,
                 payload_mass=None,
                 payload_cog=None,
                 joints_init=None,
                 joints_init_speed=1.05,
                 soft_real_time=False,
                 verbose=False,
                 receive_keys=None,
                 get_max_k=128):
        super().__init__(name="ABBInterpolationController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.lookahead_time = lookahead_time
        self.gain = gain
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.launch_timeout = launch_timeout
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        self.verbose = verbose

        # Build input queue
        example = {
            'cmd': Command.SERVOL.value,
            'target_pose': np.zeros((6,), dtype=np.float64),  # 6D pose: [x, y, z, ex, ey, ez]
            'duration': 0.0,
            'target_time': 0.0
        }
        self.input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=256
        )

        # Build ring buffer
        if receive_keys is None:
            receive_keys = [
                'ActualTCPPose',
                'ActualQ',
                'TargetTCPPose',
                'TargetQ'
            ]

        example = {
            'ActualTCPPose': np.zeros((6,), dtype=np.float64),  # TCP pose is 6D
            'ActualQ': np.zeros((7,), dtype=np.float64),        # Joint state is 7D
            'TargetTCPPose': np.zeros((6,), dtype=np.float64),  # Target TCP pose is 6D
            'TargetQ': np.zeros((7,), dtype=np.float64),        # Target joint state is 7D
            'robot_receive_timestamp': np.zeros((), dtype=np.float64)  # Timestamp is scalar
        }

        example['robot_receive_timestamp'] = time.time()
        self.ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()

    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[ABBInterpolationController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {'cmd': Command.STOP.value}
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()

    def stop_wait(self):
        self.join()

    @property
    def is_ready(self):
        return self.ready_event.is_set()
    
    def get_state(self):
        """
        获取机器人最新状态。
        返回一个字典，包括：
        - 'ActualTCPPose': 末端执行器的姿态 [x, y, z, ex, ey, ez] (单位：毫米和degree)
        - 'ActualQ': 当前关节角度 [r1, r2, r3, r4, r5, r6, r7] (单位：degree角度)
        - 'TargetTCPPose': 当前目标末端执行器姿态
        - 'TargetQ': 当前目标关节角度
        - 'robot_receive_timestamp': 数据接收的时间戳
        """
        # 调用 ring_buffer.get() 获取最新的状态
        try:
            latest_data = self.ring_buffer.get()
            return {key: latest_data[key].copy() for key in latest_data}
        except Exception as e:
            print(f"[ABBInterpolationController] Failed to get state: {e}")
            return None

    def get_all_state(self):
        """
        获取机器人所有历史状态。
        返回一个字典，包含 ring_buffer 中存储的所有状态数据。
        """
        try:
            # 从 ring_buffer 获取所有状态数据
            all_data = self.ring_buffer.get_all()
            return {key: all_data[key].copy() for key in all_data}
        except Exception as e:
            print(f"[ABBInterpolationController] Failed to get all state: {e}")
            return None
        

    def servoL(self, pose, duration=0.1):
        """
        Send a servoL command to move the robot to a specific pose.
        pose: [x, y, z, ex, ey, ez] (mm and degree)
        """
        assert self.is_alive()
        assert duration >= (1 / self.frequency)
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SERVOL.value,
            'target_pose': pose,
            'duration': duration
        }
        self.input_queue.put(message)

    def run(self):
        # try:
            print("[ABBInterpolationController] Starting run()...")
            robot = Robot(ip=self.robot_ip)

            dt = 1.0 / self.frequency
            curr_pose = robot.getActualTCPPose()  # 获取 TCP 位姿
            # print("run: curr_pose: ", curr_pose)

            curr_time = time.monotonic() 
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_time],
                poses=[curr_pose]
            )
            
            iter_idx = 0
            keep_running = True
            while keep_running:
                t_start = robot.initPeriod()
                t_now = time.monotonic()
                pose_command = pose_interp(t_now) # numpy array 
                print(f"RUN: pose_command-- content: {pose_command}")

                # position, quaternion = pose_command[:3], pose_command[3:]
                # quaternion = normalize_quaternion(quaternion)
                # robot_pose = [position.tolist(), quaternion.tolist()]

                # robot.servoL(pose_command, vel=self.max_pos_speed, acc=0.5, dt=dt,
                #             lookahead_time=self.lookahead_time, gain=self.gain)
                robot.set_cartesian(pose_command)

                state = {
                    'ActualTCPPose': np.hstack(robot.getActualTCPPose()),  # Shape (6,)
                    'ActualQ': np.array(robot.getActualQ()),  # Shape (7,)
                    'TargetTCPPose': np.array(robot.getTargetTCPPose()),  # Shape (6,)
                    'TargetQ': np.array(robot.getTargetQ()),  # Shape (7,)
                    'robot_receive_timestamp': time.time()  # Scalar
                }


                # print(f"[DEBUG] State Details:")
                # for key, value in state.items():
                #     print(f"  {key}: {value}, shape: {np.shape(value)}, type: {type(value)}")
                    
                self.ring_buffer.put(state)

                try:
                    commands = self.input_queue.get_all()
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                for i in range(n_cmd):
                    command = {key: commands[key][i] for key in commands}
                    cmd = command['cmd']

                    if cmd == Command.STOP.value:
                        keep_running = False
                        break
                    elif cmd == Command.SERVOL.value:
                        # since curr_pose always lag behind curr_target_pose
                        # if we start the next interpolation with curr_pose
                        # the command robot receive will have discontinouity 
                        # and cause jittery robot behavior.
                        target_pose = command['target_pose']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        pose_interp = pose_interp.drive_to_waypoint(
                            pose=target_pose,
                            time=t_insert,
                            curr_time=curr_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print("[RTDEPositionalController] New pose target:{} duration:{}s".format(
                                target_pose, duration))
                    
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']
                        target_time = float(command['target_time'])
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                robot.waitPeriod(t_start)

                if iter_idx == 0:
                    self.ready_event.set()  # 通知父进程子进程启动成功
                iter_idx += 1

                if self.verbose:
                    print("[ABBInterpolationController] Exiting main loop.")

        # except Exception as e:
        #     print(f"[ABBInterpolationController] Exception in run(): {e}")
        # finally:
        #     self.ready_event.set()
        #     print("[ABBInterpolationController] Cleaning up.")