# # -*- coding: UTF-8 -*-

import socket
import json 
import time
import inspect
from threading import Thread
from collections import deque
import logging
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp as Rslerp
from queue import Queue

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())
    
class Robot:
    def __init__(self, 
                 ip          = '192.168.125.1', 
                 port_motion = 5000,
                 port_logger = 5002):

        # 初始化控制周期相关变量
        self.t_start = None
        self.control_period = 0.002  # 控制周期，单位秒

        self.delay   = 0.08  # the frequency for control is ~10 Hz
        self.pose = Queue()  # 使用线程安全的队列存储 TCP 位姿
        self.joints = Queue()  # 使用线程安全的队列存储关节角度
        self.target_pose = Queue()  # 队列存储目标 TCP 位姿
        self.target_joints = Queue()  # 队列存储目标关节角度

        self.start_logger_thread(ip, port_logger)
        self.connect_motion((ip, port_motion))
        
        self.set_units('millimeters', 'degrees')
        self.set_tool()
        self.set_workobject()
        self.set_speed()
        self.set_zone()

    def connect_motion(self, remote):        
        log.info('Attempting to connect to robot motion server at %s', str(remote))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.5)
        self.sock.connect(remote)
        self.sock.settimeout(None)
        log.info('Connected to robot motion server at %s', str(remote))

    def connect_logger(self, remote, maxlen=None):
        self.pose   = deque(maxlen=maxlen)
        self.joints = deque(maxlen=maxlen)
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(remote)
        s.setblocking(1)
        try:
            while True:
                data = map(float, s.recv(4096).split())
                if   int(data[1]) == 0: 
                    self.pose.append([data[2:5], data[5:]])
                #elif int(data[1]) == 1: self.joints.append([a[2:5], a[5:]])
        finally:
            s.shutdown(socket.SHUT_RDWR)

    def set_units(self, linear, angular):
        units_l = {'millimeters': 1.0,
                   'meters'     : 1000.0,
                   'inches'     : 25.4}
        units_a = {'degrees' : 1.0,
                   'radians' : 57.2957795}
        self.scale_linear = units_l[linear]
        self.scale_angle  = units_a[angular]

    def set_cartesian(self, pose):
        '''
        Executes a move immediately from the current pose,
        to 'pose', with units of millimeters.
        '''
        x, y, z, ez, ey, ex = pose  # unpack
        pose = [x, y, z, ex, ey, ez]  # rearrange
        msg  = "01 " + self.format_pose(pose)   
        return self.send(msg)

    def set_joints(self, joints):
        '''
        Executes a move immediately, from current joint angles,
        to 'joints', in degrees. 
        '''
        if len(joints) != 7: return False
        msg = "02 "
        for joint in joints: msg += format(joint*self.scale_angle, "+08.2f") + " " 
        msg += "#" 
        return self.send(msg)

    def get_cartesian(self):
        '''
        Returns the current pose of the robot, in millimeters and degrees
        '''
        msg = "03 #"
        data = self.send(msg).split()
        r = [float(s) for s in data]
        return [r[2:8]]

    def get_joints(self):
        '''
        Returns the current angles of the robots joints, in degrees. 
        '''
        msg = "04 #"
        data = self.send(msg).split()
        return [float(s) / self.scale_angle for s in data[2:9]]

    def get_external_axis(self):
        '''
        If you have an external axis connected to your robot controller
        (such as a FlexLifter 600, google it), this returns the joint angles
        '''
        msg = "05 #"
        data = self.send(msg).split()
        return [float(s) for s in data[2:9]]
       
    def get_robotinfo(self):
        '''
        Returns a robot- unique string, with things such as the
        robot's model number. 
        Example output from and IRB 2400:
        ['24-53243', 'ROBOTWARE_5.12.1021.01', '2400/16 Type B']
        '''
        msg = "98 #"
        data = str(self.send(msg))[5:].split('*')
        log.debug('get_robotinfo result: %s', str(data))
        return data
    
    def start_logger_thread(self, ip, port_logger):
        log_thread = Thread(target=self.get_net, args=(ip, port_logger), daemon=True)
        log_thread.start()


    def get_net(self, ip, port):
        """
        Connects to the robot's logger server, receives data, and parses it.
        """
        # Create TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.5)  # Set connection timeout

        try:
            # Connect to the specified IP and port
            sock.connect((ip, port))
            log.info(f"Connected to robot logger server at {ip}:{port}")
            sock.settimeout(None)  # Switch to blocking mode

            buffer = ""  # Buffer for received data
            while True:
                try:
                    # Receive raw data from the socket
                    raw_data = sock.recv(4096).decode('utf-8')
                    buffer += raw_data  # Append received data to buffer

                    # Process complete messages ending with "@"
                    while "@" in buffer:
                        # Extract complete message and update buffer
                        complete_message, buffer = buffer.split("@", 1)
                        complete_message = complete_message.strip()

                        # Skip empty or malformed messages
                        if not complete_message or "|#1|" not in complete_message:
                            log.warning(f"Invalid or incomplete message: {complete_message}")
                            continue

                        # Parse TCP and Joint State data
                        try:
                            # Split the message into TCP (#0) and Joint State (#1) sections
                            tcp_section, joint_section = complete_message.split("|#1|", 1)
                            tcp_section = tcp_section.replace("#0|", "").strip()  # Clean up #0 marker
                            joint_section = joint_section.strip()  # Strip any extra whitespace

                            # Parse TCP data (#0): Expecting 6 floating-point values
                            tcp_values = list(map(float, tcp_section.split(",")))
                            if len(tcp_values) != 6:
                                raise ValueError(f"Invalid TCP data length: {tcp_values}")
                            
                            # Parse Joint State data (#1): Expecting 7 floating-point values
                            joint_values = list(map(float, joint_section.split(",")))
                            if len(joint_values) != 7:
                                raise ValueError(f"Invalid Joint State data length: {joint_values}")

                            # Add TCP data to the pose queue
                            if not self.pose.full():
                                self.pose.put(tcp_values)
                            else:
                                self.pose.get()  # Drop the oldest data
                                self.pose.put(tcp_values)

                            # Add Joint State data to the joints queue
                            if not self.joints.full():
                                self.joints.put(joint_values)
                            else:
                                self.joints.get()  # Drop the oldest data
                                self.joints.put(joint_values)

                            # Log parsed data (optional: for debugging purposes)
                            log.info(f"TCP Data: {tcp_values}")
                            log.info(f"Joint Data: {joint_values}")

                        except ValueError as e:
                            log.warning(f"Error parsing data: {e}. Message: {complete_message}")
                            continue

                except Exception as e:
                    log.error(f"Error receiving or processing data: {e}")
                    break

        except Exception as e:
            log.error(f"Failed to connect to {ip}:{port}: {e}")

        finally:
            sock.close()
            log.info("Socket connection closed.")

    def initPeriod(self):
        """
        初始化时间基准，返回当前时间戳
        """
        self.t_start = time.time()
        return self.t_start


    def waitPeriod(self, t_start):
        """
        模拟 waitPeriod 功能，等待当前周期结束
        """
        # 计算下一个周期的起始时间
        next_period_start = t_start + self.control_period

        # 获取当前时间
        now = time.time()

        # 如果当前时间比下一个周期时间早，则休眠
        if now < next_period_start:
            time.sleep(next_period_start - now)

        # 如果时间已经超过了下一个周期（可能是延迟），直接返回


    def servoL_quaternions(self, pose_command, vel, acc, dt, lookahead_time, gain):
        """
        实现类似 UR 的 servoL 功能：
        - pose_command: [[x, y, z], [q1, q2, q3, q4]] 毫米/四元数
        - vel:          期望的速度（未使用）
        - acc:          期望的加速度（未使用）
        - dt:           控制时间间隔（秒）
        - lookahead_time: 平滑运动的前瞻时间（未使用）
        - gain:         伺服增益（未使用）

        注意：vel, acc, lookahead_time, gain 是占位参数，实际运动由 dt 控制。
        """

        # 获取当前 TCP 位姿
        curr_pose = self.get_cartesian_t2()  # [[x, y, z], [qx, qy, qz, qw]]
        curr_pose = [[float(x) for x in group] for group in curr_pose]
        if curr_pose == [[0, 0, 0], [0, 0, 0, 0]]:
            raise ValueError("Failed to get current TCP pose")
        # 分解当前位置
        curr_position, curr_quaternion = curr_pose  # [x, y, z], [q1, q2, q3, q4]
        # 解析目标位置和旋转
        target_position = np.array(pose_command[0]) # 毫米
        target_quaternion = np.array(pose_command[1])  # 四元数 [q1, q2, q3, q4]

        # 检查目标四元数是否为单位四元数
        if not np.isclose(np.linalg.norm(target_quaternion), 1.0, atol=1e-6):
            raise ValueError("Target quaternion is not normalized!")

        # 插值步数（基于 dt 和控制周期）
        steps = max(2, int(dt / self.control_period))  # 确保至少有 2 步

        # 线性插值位置
        pos_traj = np.linspace(curr_position, target_position, steps)

        # 使用 Slerp 进行四元数插值
        key_times = np.array([0, 1])  # 插值关键时间点
        key_rots = R.from_quat([curr_quaternion, target_quaternion])  # 创建 Rotation 对象
        slerp = Rslerp(key_times, key_rots)  # 创建 Slerp 对象
        t_array = np.linspace(0, 1, steps)  # 插值时间点
        quat_traj = slerp(t_array)  # 获取插值的旋转对象

        # 实时发送插值点
        for i in range(steps):
            # 获取插值后的位置
            interpolated_position = pos_traj[i]

            # 获取插值后的旋转（四元数）
            interpolated_quaternion = quat_traj[i].as_quat()

            # 组合成 [[x, y, z], [q1, q2, q3, q4]] 格式
            interpolated_pose = [
                interpolated_position.tolist(),  # [x, y, z]
                interpolated_quaternion.tolist()  # [q1, q2, q3, q4]
            ]

            # 调试打印
            # print(f"[DEBUG] Interpolated Pose {i}: {interpolated_pose}")

            # 发送运动指令
            self.set_cartesian(interpolated_pose)

            # 等待下一个周期
            time.sleep(self.control_period)


    def servoL(self, pose_command, vel, acc, dt, lookahead_time, gain):
        """
        实现类似 UR 的 servoL 功能，输入为欧拉角：
        - pose_command: [[x, y, z], [rx, ry, rz]] 毫米/欧拉角 (弧度)
        - vel:          期望的速度（未使用）
        - acc:          期望的加速度（未使用）
        - dt:           控制时间间隔（秒）
        - lookahead_time: 平滑运动的前瞻时间（未使用）
        - gain:         伺服增益（未使用）

        注意：vel, acc, lookahead_time, gain 是占位参数，实际运动由 dt 控制。
        """

        # print(f"ABBPY: target_pose: type: {type(pose_command)}, content: {pose_command}")
        # 获取当前 TCP 位姿
        curr_pose = self.getActualTCPPose()  # [x, y, z, ex, ey, ez]
        # print(f"ABBPY: curr_pose: type: {type(curr_pose)}, content: {curr_pose}")

        if curr_pose == [0, 0, 0, 0, 0, 0]:
            raise ValueError("Failed to get current TCP pose")
        curr_position = np.array(curr_pose[:3])  # [x, y, z]
        curr_euler = np.array(curr_pose[3:])     # [ex, ey, ez]
       
        target_position = np.array(pose_command[:3])  # [x, y, z]
        target_euler = np.array(pose_command[3:])     # [ex, ey, ez]
        
        # 插值步数（基于 dt 和控制周期）
        steps = max(2, int(dt / self.control_period))  # 确保至少有 2 步

        # 线性插值位置
        pos_traj = np.linspace(curr_position, target_position, steps)

        # 线性插值欧拉角
        euler_traj = np.linspace(curr_euler, target_euler, steps)

        # 实时发送插值点
        for i in range(steps):
            # 获取插值后的位置
            interpolated_position = pos_traj[i]

            # 获取插值后的欧拉角
            interpolated_euler = euler_traj[i]

            # 组合成 [x, y, z, ex, ey, ez] 格式
            interpolated_pose = np.concatenate([interpolated_position, interpolated_euler]).tolist()

            # 调试打印
            # print(f"[DEBUG] Interpolated Pose {i}: {interpolated_pose}")

            # 发送运动指令
            self.set_cartesian(interpolated_pose)

            # 等待下一个控制周期
            time.sleep(self.control_period)


    def getTargetTCPPose(self):
        """
        获取目标 TCP 位姿 [x, y, z, rx, ry, rz]/mm + degree
        - 位置单位：mm
        - 姿态单位：轴角（角度）
        """
        # 检查目标位姿队列是否为空
        if self.target_pose.empty():
            log.warning("No target TCP pose available")
            # return [-9.6, 192.6, 200.0, 0.066, 0.842, -0.111, 0.523]  # 如果没有目标数据，返回默认值
            return [0, 0, 0, 0, 0, 0]  # 如果没有目标数据，返回默认值

        # 从队列中获取目标位姿
        target_pose = self.target_pose.get()  # [x, y, z, ex, ey, ez]
        return target_pose
    
    def getTargetQ(self):
        """
        获取目标关节角度 [q1, q2, q3, q4, q5, q6]
        - 单位：角度
        """
        # 检查目标关节角度队列是否为空
        if self.target_joints.empty():
            log.warning("No target joint angles available")
            return [0, 0, 0, 0, 0, 0, 0]  # 如果没有目标数据，返回默认值

        # 从队列中获取目标关节角度
        target_joints = self.target_joints.get()

        # 返回目标关节角度
        return [float(q) for q in target_joints]  # 确保数据为浮点数
    
    def getActualTCPPose(self):
        """
        返回当前 TCP 位姿 [x, y, z, ex, ey, ez] 功能 和 get_cartesian_t2() 完全一致
        - 位置单位：毫米
        - 姿态单位：欧拉角
        """
        # tcp_pose = self.get_cartesian_t2()  # get the last TCP data [x, y, z, ex, ey, ez]
        # # tcp_pose = [[float(x) for x in group] for group in tcp_pose]
        # # if tcp_pose == [[0, 0, 0], [0, 0, 0, 0]]:
        # #     return [0, 0, 0, 0, 0, 0]
        # tcp_pose = list(map(float,tcp_pose))
        # return tcp_pose
    
        if self.pose.empty():
            #print("No pose data available")
            log.warning("No pose data available")
            return [0, 0, 0, 0, 0, 0]
        
        return self.pose.get()  # 从队列中获取最新数据
    

    def getActualQ(self):
        """
        返回当前关节角度 [q1, q2, q3, q4, q5, q6, q_eax] (7,)
        - 单位：轴角(角度)
        """
        if self.joints.empty():
            #print("No joints data available")
            log.warning("No joints data available")
            return [0, 0, 0, 0, 0, 0, 0]
        
        return self.joints.get()  # 从队列中获取最新数据

    
    def setTargetTCPPose(self, pose):
        """
        设置目标 TCP 位姿 [x, y, z, ex, ey, ez]。
        """
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError("Invalid pose format, expected [x, y, z, ex, ey, ez]")
        self.target_pose.put(pose)  # 将目标位姿存入队列

    def setTargetQ(self, joints):
        """
        设置目标关节角度。
        - joints: [q1, q2, q3, q4, q5, q6, q7] 格式
        """
        if not isinstance(joints, list) or len(joints) != 7:
            raise ValueError("Invalid joints format, expected [q1, q2, q3, q4, q5, q6 , q7]")
        self.target_joints.put(joints)  # 将目标关节角度存入队
        
    def set_tool(self, tool=[[0,0,0], [1,0,0,0]]):
        '''
        Sets the tool centerpoint (TCP) of the robot. 
        When you command a cartesian move, 
        it aligns the TCP frame with the requested frame.
        
        Offsets are from tool0, which is defined at the intersection of the
        tool flange center axis and the flange face.
        '''
        msg       = "06 " + self.format_pose(tool)    
        self.send(msg)
        self.tool = tool

    def load_json_tool(self, file_obj):
        if file_obj.__class__.__name__ == 'str':
            file_obj = open(filename, 'rb');
        tool = check_coordinates(json.load(file_obj))
        self.set_tool(tool)
        
    def get_tool(self): 
        log.debug('get_tool returning: %s', str(self.tool))
        return self.tool

    def set_workobject(self, work_obj=[[0,0,0],[1,0,0,0]]):
        '''
        The workobject is a local coordinate frame you can define on the robot,
        then subsequent cartesian moves will be in this coordinate frame. 
        '''
        msg = "07 " + self.format_pose(work_obj)   
        self.send(msg)

    def set_speed(self, speed=[100,50,50,50]):
        '''
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        '''

        if len(speed) != 4: return False
        msg = "08 " 
        msg += format(speed[0], "+08.1f") + " " 
        msg += format(speed[1], "+08.2f") + " "  
        msg += format(speed[2], "+08.1f") + " " 
        msg += format(speed[3], "+08.2f") + " #"     
        self.send(msg)

    def set_zone(self, 
                 zone_key     = 'z1', 
                 point_motion = False, 
                 manual_zone  = []):
        zone_dict = {'z0'  : [.3,.3,.03], 
                    'z1'  : [1,1,.1], 
                    'z5'  : [5,8,.8], 
                    'z10' : [10,15,1.5], 
                    'z15' : [15,23,2.3], 
                    'z20' : [20,30,3], 
                    'z30' : [30,45,4.5], 
                    'z50' : [50,75,7.5], 
                    'z100': [100,150,15], 
                    'z200': [200,300,30]}
        '''
        Sets the motion zone of the robot. This can also be thought of as
        the flyby zone, AKA if the robot is going from point A -> B -> C,
        how close do we have to pass by B to get to C
        
        zone_key: uses values from RAPID handbook (stored here in zone_dict)
        with keys 'z*', you should probably use these

        point_motion: go to point exactly, and stop briefly before moving on

        manual_zone = [pzone_tcp, pzone_ori, zone_ori]
        pzone_tcp: mm, radius from goal where robot tool centerpoint 
                   is not rigidly constrained
        pzone_ori: mm, radius from goal where robot tool orientation 
                   is not rigidly constrained
        zone_ori: degrees, zone size for the tool reorientation
        '''

        if point_motion: 
            zone = [0,0,0]
        elif len(manual_zone) == 3: 
            zone = manual_zone
        elif zone_key in zone_dict.keys(): 
            zone = zone_dict[zone_key]
        else: return False
        
        msg = "09 " 
        msg += str(int(point_motion)) + " "
        msg += format(zone[0], "+08.4f") + " " 
        msg += format(zone[1], "+08.4f") + " " 
        msg += format(zone[2], "+08.4f") + " #" 
        self.send(msg)

    def buffer_add(self, pose):
        '''
        Appends single pose to the remote buffer
        Move will execute at current speed (which you can change between buffer_add calls)
        '''
        msg = "30 " + self.format_pose(pose) 
        self.send(msg)

    def buffer_set(self, pose_list):
        '''
        Adds every pose in pose_list to the remote buffer
        '''
        self.clear_buffer()
        for pose in pose_list: 
            self.buffer_add(pose)
        if self.buffer_len() == len(pose_list):
            log.debug('Successfully added %i poses to remote buffer', 
                      len(pose_list))
            return True
        else:
            log.warn('Failed to add poses to remote buffer!')
            self.clear_buffer()
            return False

    def clear_buffer(self):
        msg = "31 #"
        data = self.send(msg)
        if self.buffer_len() != 0:
            log.warn('clear_buffer failed! buffer_len: %i', self.buffer_len())
            raise NameError('clear_buffer failed!')
        return data

    def buffer_len(self):
        '''
        Returns the length (number of poses stored) of the remote buffer
        '''
        msg = "32 #"
        data = self.send(msg).split()
        return int(float(data[2]))

    def buffer_execute(self):
        '''
        Immediately execute linear moves to every pose in the remote buffer.
        '''
        msg = "33 #"
        return self.send(msg)

    def set_external_axis(self, axis_unscaled=[-550,0,0,0,0,0]):
        if len(axis_values) != 6: return False
        msg = "34 "
        for axis in axis_values:
            msg += format(axis, "+08.2f") + " " 
        msg += "#"   
        return self.send(msg)

    def move_circular(self, pose_onarc, pose_end):
        '''
        Executes a movement in a circular path from current position, 
        through pose_onarc, to pose_end
        '''
        msg_0 = "35 " + self.format_pose(pose_onarc)  
        msg_1 = "36 " + self.format_pose(pose_end)

        data = self.send(msg_0).split()
        if data[1] != '1': 
            log.warn('move_circular incorrect response, bailing!')
            return False
        return self.send(msg_1)

    def set_dio(self, value, id=0):
        '''
        A function to set a physical DIO line on the robot.
        For this to work you're going to need to edit the RAPID function
        and fill in the DIO you want this to switch. 
        '''
        msg = '97 ' + str(int(bool(value))) + ' #'
        return 
        #return self.send(msg)
        
    def send(self, message, wait_for_response=True):
        '''
        Send a formatted message to the robot socket.
        if wait_for_response, we wait for the response and return it
        '''
        caller = inspect.stack()[1][3]
        log.debug('%-14s sending: %s', caller, message)
        self.sock.send(message.encode('utf-8'))
        time.sleep(self.delay)
        if not wait_for_response: return
        data = self.sock.recv(4096)
        log.debug('%-14s recieved: %s', caller, data)
        return data
        
    def format_pose(self, pose):
        pose = check_coordinates(pose)
        msg  = ''
        if len(pose) == 2:
            for cartesian in pose[0]:
                msg += format(cartesian * self.scale_linear,  "+08.1f") + " " 
            for quaternion in pose[1]:
                msg += format(quaternion, "+08.5f") + " " 
        # print("pose: ", pose)
        if len(pose) == 1:
            for cartesian in pose[0][0:3]:
                # print("cartesian: ", cartesian)
                msg += format(cartesian * self.scale_linear,  "+08.2f") + " " 
            for quaternion in pose[0][3:6]:
                msg += format(quaternion * self.scale_angle, "+08.2f") + " " 

        msg += "#" 
        return msg       
        
    def close(self):
        self.send("99 #", False)
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        log.info('Disconnected from ABB robot.')

    def __enter__(self):
        return self
        
    def __exit__(self, type, value, traceback):
        self.close()

def check_coordinates(coordinates):
    if ((len(coordinates) == 2) and 
        (len(coordinates[0]) == 3) and 
        (len(coordinates[1]) == 4)): 
        return coordinates
    # elif (len(coordinates) == 7):
    #     return [coordinates[0:3], coordinates[3:7]]
    elif (len(coordinates) == 6):
        return [coordinates[:]]
    log.warn('Recieved malformed coordinate: %s', str(coordinates))
    raise NameError('Malformed coordinate!')

if __name__ == '__main__':
    formatter = logging.Formatter("[%(asctime)s] %(levelname)-7s (%(filename)s:%(lineno)3s) %(message)s", "%Y-%m-%d %H:%M:%S")
    handler_stream = logging.StreamHandler()
    handler_stream.setFormatter(formatter)
    handler_stream.setLevel(logging.DEBUG)
    log = logging.getLogger('abb')
    log.setLevel(logging.DEBUG)
    log.addHandler(handler_stream)
    
