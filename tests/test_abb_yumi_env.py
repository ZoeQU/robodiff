import sys

project_path = "/home/flair/Projects/robodiff/"
if project_path not in sys.path:
    sys.path.insert(0, project_path)

import time
import numpy as np
from diffusion_policy.real_world.real_yumi_env import RealEnv

def test_yumi_env():
    # 配置参数
    output_dir = "./test_output"
    robot_ip = "192.168.125.108"  # 替换为 YuMi 的实际 IP
    frequency = 10
    obs_image_resolution = (640, 480)

    # 创建环境
    env = RealEnv(
        output_dir=output_dir,
        robot_ip=robot_ip,
        frequency=frequency,
        obs_image_resolution=obs_image_resolution,
        init_joints=True
    )
    
    # 启动环境
    with env:
        # 测试获取观察值
        for i in range(5):
            obs = env.get_obs()
            # print(f"Step {i}: Observation:", obs)
            print(f"Step {i}")
            time.sleep(1 / frequency)

        # 测试执行动作
        actions = np.array([
            [3.299e+02, 3.954e+02, 2.360e+02, 2.800e-02, 9.980e-01, 5.100e-02,
        3.800e-02],  # 假设的目标姿态
            [3.569e+02, 3.453e+02, 2.360e+02, 2.800e-02, 9.980e-01, 5.100e-02,
        3.800e-02]  # 假设的目标姿态
        ])
        timestamps = np.array([time.time() + 0.5, time.time() + 1.0])
        env.exec_actions(actions=actions, timestamps=timestamps)

        # 等待动作执行完成
        time.sleep(2)

        # 测试结束一段记录
        env.start_episode()
        time.sleep(2)
        env.end_episode()

        print("Test finished!")

if __name__ == "__main__":
    test_yumi_env()