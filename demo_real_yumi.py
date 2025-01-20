"""
Usage:
(robodiff)$ python yumi_demo.py -o <demo_save_dir> --robot_ip <ip_of_yumi>

Robot movement:
Move your SpaceMouse to move the robot EEF (locked in xy plane).
Press SpaceMouse right button to unlock z axis.
Press SpaceMouse left button to enable rotation axes.

Recording control:
Click the OpenCV window (make sure it's in focus).
Press "C" to start recording.
Press "S" to stop recording.
Press "Q" to exit the program.
Press "Backspace" to delete the previously recorded episode.
"""

# %%
import time
from multiprocessing.managers import SharedMemoryManager
import click
import cv2
import numpy as np
import scipy.spatial.transform as st
from diffusion_policy.real_world.real_yumi_env import RealEnv  
from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse
from diffusion_policy.common.precise_sleep import precise_wait
from diffusion_policy.real_world.keystroke_counter import (
    KeystrokeCounter, Key, KeyCode
)

@click.command()
@click.option('--output', '-o', required=True, help="Directory to save demonstration dataset.")
@click.option('--robot_ip', '-ri', required=True, help="YuMi's IP address e.g. 192.168.125.1")
@click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")
@click.option('--init_joints', '-j', is_flag=True, default=False, help="Whether to initialize robot joint configuration in the beginning.")
@click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")
@click.option('--command_latency', '-cl', default=0.01, type=float, help="Latency between receiving SpaceMouse command to executing on Robot in Sec.")
def main(output, robot_ip, vis_camera_idx, init_joints, frequency, command_latency):
    """
    主函数，用于控制 ABB YuMi 并采集演示数据。

    **输入**:
    - `output`: `str`，保存数据的目录路径。
    - `robot_ip`: `str`，YuMi 的 IP 地址。
    - `vis_camera_idx`: `int`，要显示的 Realsense 相机索引。
    - `init_joints`: `bool`，是否初始化关节。
    - `frequency`: `float`，机器人控制频率（Hz）。
    - `command_latency`: `float`，接收 SpaceMouse 命令到机器人执行的延迟（秒）。

    **输出**:
    - 无返回值，但将采集的数据保存到指定目录。
    """
    dt = 1 / frequency
    with SharedMemoryManager() as shm_manager:
        with KeystrokeCounter() as key_counter, \
            Spacemouse(shm_manager=shm_manager) as sm, \
            RealEnv(
                output_dir=output,
                robot_ip=robot_ip,

                obs_image_resolution=(1280, 720),
                frequency=frequency,
                init_joints=True,
                enable_multi_cam_vis=True,
                record_raw_video=True,
                thread_per_video=3,
                video_crf=21,
                shm_manager=shm_manager
            ) as env:
            cv2.setNumThreads(1)

            # 设置 Realsense 参数
            env.realsense.set_exposure(exposure=120, gain=0)
            env.realsense.set_white_balance(white_balance=5900)

            # 初始化机器人状态
            time.sleep(1.0)
            print('Ready!')
            state = env.get_robot_state()
            target_pose = state['ActualTCPPose']  # shape (6,)
            print(f"Initial target_pose: {target_pose}, shape: {target_pose.shape}")
            t_start = time.monotonic()
            iter_idx = 0
            stop = False
            is_recording = False

            while not stop:
                # 时间控制
                t_cycle_end = t_start + (iter_idx + 1) * dt
                t_sample = t_cycle_end - command_latency
                t_command_target = t_cycle_end + dt

                # 获取当前观察值
                obs = env.get_obs()

                # 处理按键事件
                press_events = key_counter.get_press_events()
                for key_stroke in press_events:
                    if key_stroke == KeyCode(char='q'):
                        stop = True  # 退出程序
                    elif key_stroke == KeyCode(char='c'):
                        # 开始录制
                        env.start_episode(t_start + (iter_idx + 2) * dt - time.monotonic() + time.time())
                        key_counter.clear()
                        is_recording = True
                        print('Recording!')
                    elif key_stroke == KeyCode(char='s'):
                        # 停止录制
                        env.end_episode()
                        key_counter.clear()
                        is_recording = False
                        print('Stopped.')
                    elif key_stroke == Key.backspace:
                        # 删除最近录制的 episode
                        if click.confirm('Are you sure to drop an episode?'):
                            env.drop_episode()
                            key_counter.clear()
                            is_recording = False

                # 处理阶段标记（通过空格键）
                stage = key_counter[Key.space]

                # 可视化
                vis_img = obs[f'camera_{vis_camera_idx}'][-1, :, :, ::-1].copy()
                episode_id = env.replay_buffer.n_episodes
                text = f'Episode: {episode_id}, Stage: {stage}'
                if is_recording:
                    text += ', Recording!'
                cv2.putText(
                    vis_img,
                    text,
                    (10, 30),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1,
                    thickness=2,
                    color=(255, 255, 255)
                )

                cv2.imshow('default', vis_img)
                cv2.pollKey()

                precise_wait(t_sample)
                # get teleop command # TODO 获取 SpaceMouse 输入
                sm_state = sm.get_motion_state_transformed()
                sm_state = sm_state * 100
                dpos = sm_state[:3] * (env.max_pos_speed / frequency) # 位置增量
                drot_xyz = sm_state[3:] * (env.max_rot_speed / frequency)   # 旋转增量（欧拉角）

                # print("Demo: dpos ", dpos)
                
                if not sm.is_button_pressed(0):  # 按钮 0 控制模式切换
                    # translation mode
                    drot_xyz[:] = 0
                else:
                    dpos[:] = 0
                if not sm.is_button_pressed(1):  # 按钮 1 控制 Z 轴锁定
                    # 2D translation mode
                    dpos[2] = 0    
                
                drot = st.Rotation.from_euler('xyz', drot_xyz)
                target_pose[:3] += dpos
                target_pose[3:] = (drot * st.Rotation.from_rotvec(
                    target_pose[3:])).as_rotvec()
                                
                # execute teleop command
                env.exec_actions(
                    actions=[np.array(target_pose)],  # target pose [x, y, z, ex, ey, ez]
                    timestamps=[t_command_target - time.monotonic() + time.time()],  # timestamp
                    stages=[stage]  # stage
                )

                precise_wait(t_cycle_end)
                iter_idx += 1

if __name__ == '__main__':
    main()