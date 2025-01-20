import sys

project_path = "/home/flair/Projects/robodiff/"
if project_path not in sys.path:
    sys.path.insert(0, project_path)

import time
from multiprocessing.managers import SharedMemoryManager
import numpy as np
from diffusion_policy.real_world.abb_controller import ABBInterpolationController, Command
from diffusion_policy.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
import time
import numpy as np


# def test_controller_startup(robot_ip):
#     shm_manager = SharedMemoryManager()
#     shm_manager.start()

#     try:
#         controller = ABBInterpolationController(
#             shm_manager=shm_manager,
#             robot_ip=robot_ip,
#             frequency=125,
#             verbose=True
#         )

#         print("Starting ABB Controller...")
#         controller.start(wait=True)

#         if controller.is_ready:
#             print("ABB Controller is ready!")
#         else:
#             print("ABB Controller failed to start.")

#         print("Stopping ABB Controller...")
#         controller.stop(wait=True)

#     finally:
#         shm_manager.shutdown()

# if __name__ == "__main__":
#     robot_ip = "192.168.125.108"  # Replace with your ABB robot IP
#     test_controller_startup(robot_ip)


def test_abb_controller():
    shm_manager = SharedMemoryManager()
    shm_manager.start()

    # 创建 ABB Controller
    robot_ip = "192.168.125.108"  # 替换为实际 ABB YuMi 的 IP 地址
    controller = ABBInterpolationController(
        shm_manager=shm_manager,
        robot_ip=robot_ip,
        frequency=125,
        lookahead_time=0.1,
        gain=300,
        max_pos_speed=100,
        max_rot_speed=30,
        receive_keys=None,
        verbose=True
    )

    # 启动控制器
    controller.start(wait=True)

    try:
        # 等待控制器准备就绪
        time.sleep(1)

        # 测试 get_state()
        state = controller.get_state()
        print("[Test] Current Robot State:")
        print(state)

        # 测试 get_all_state()
        all_states = controller.get_all_state()
        print("[Test] All Robot States from Ring Buffer:")
        for key, value in all_states.items():
            print(f"{key}: {value.shape}, dtype={value.dtype}")

    finally:
        # 停止控制器
        controller.stop(wait=True)
        shm_manager.shutdown()

if __name__ == "__main__":
    test_abb_controller()
