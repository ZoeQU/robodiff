import threading
import queue
import test_abb2 as abb
import time
import os
import numpy as np
from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent


# Spacemouse 类
class Spacemouse:
    def __init__(self, max_value=500, deadzone=0.01):
        self.max_value = max_value
        self.deadzone = deadzone
        self.translation = np.zeros(3)  # [x, y, z]
        self.running = False

    def start(self):
        spnav_open()
        self.running = True

    def stop(self):
        self.running = False
        spnav_close()

    def poll_event(self):
        if not self.running:
            return None

        event = spnav_poll_event()
        if isinstance(event, SpnavMotionEvent):
            # Normalize and apply deadzone
            motion = np.array(event.translation) / self.max_value
            motion[np.abs(motion) < self.deadzone] = 0
            self.translation = motion[:3]  # Only consider translation

    def get_motion_state(self):
        return self.translation


# 定义共享队列
motion_queue = queue.Queue()


# SpaceMouse 监听线程
def listen_spacemouse(spacemouse):
    spacemouse.start()
    last_time = time.time()
    try:
        while True:
            spacemouse.poll_event()
            motion_state = spacemouse.get_motion_state()
            # if not np.all(motion_state == 0):  # 如果有有效输入
            #     # 将输入放入队列
            #     motion_queue.put(motion_state)
            # time.sleep(0.01)  # 降低 CPU 占用率
            current_time = time.time()

            # 每隔 0.1 秒（10Hz）采样一次
            if current_time - last_time >= 0.1:
                if not np.all(motion_state == 0):
                    motion_queue.put(motion_state)
                last_time = current_time

            time.sleep(0.01)  # 降低 CPU 占用率
    finally:
        spacemouse.stop()


# 机械臂控制线程
def control_robot(robot, move_distance):
    current_pose = robot.get_cartesian()
    while True:
        try:
            # 从队列中获取 SpaceMouse 输入
            motion_state = motion_queue.get(timeout=1)  # 如果超过 1 秒无输入，会抛出异常
            
            dpos = np.zeros(3)

            # 根据输入计算目标增量
            if motion_state[0] > 0:  # X 轴
                dpos[0] = move_distance
            elif motion_state[0] < 0:
                dpos[0] = -move_distance
            if motion_state[1] > 0:  # Y 轴
                dpos[1] = move_distance
            elif motion_state[1] < 0:
                dpos[1] = -move_distance
            # if motion_state[2] > 0:  # Z 轴
            #     dpos[2] = move_distance
            # elif motion_state[2] < 0:
            #     dpos[2] = -move_distance

            # 更新目标位姿
            current_pose[0][0] += dpos[0]
            current_pose[0][1] += dpos[1]
            current_pose[0][2] += dpos[2]

            # 发送目标位姿给机械臂
            robot.set_cartesian(current_pose)
            print(f"Moved to: {current_pose}")

        except queue.Empty:
            # 如果队列为空，继续等待输入
            continue


# 主函数
def main():
    # 初始化 Spacemouse 和机械臂
    spacemouse = Spacemouse(deadzone=0.05)
    robot = abb.Robot(ip='192.168.125.1', port_motion=5000)
    print("Robot connected!")

    # 定义移动距离
    move_distance = 1  # 每次移动的距离（单位 mm）

    # 启动监听线程
    listener_thread = threading.Thread(target=listen_spacemouse, args=(spacemouse,))
    listener_thread.daemon = True  # 设置为守护线程，主线程退出时自动结束
    listener_thread.start()

    # 启动控制线程
    control_robot(robot, move_distance)


if __name__ == "__main__":
    main()
