import sys

project_path = "/home/flair/Projects/robodiff/"
if project_path not in sys.path:
    sys.path.insert(0, project_path)

import time
import numpy as np
from multiprocessing.managers import SharedMemoryManager
from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse



def test_spacemouse():
    # 初始化共享內存管理器
    shm_manager = SharedMemoryManager()
    shm_manager.start()

    # 初始化 SpaceMouse
    sm = Spacemouse(shm_manager=shm_manager)
    sm.start()

    print("請移動 SpaceMouse，按下按鈕以測試輸入數據...")
    try:
        while True:
            # 獲取 SpaceMouse 的運動狀態
            sm_state = sm.get_motion_state_transformed()
            # 獲取按鈕狀態
            button_pressed = sm.is_button_pressed(0), sm.is_button_pressed(1)

            # 打印輸入數據
            print(f"Motion State: {sm_state}")
            print(f"Button Pressed: {button_pressed}")

            # 每 0.1 秒打印一次
            time.sleep(0.1)
    except KeyboardInterrupt:
        # 結束測試
        print("測試結束。")
    finally:
        # 停止 SpaceMouse 並釋放資源
        sm.stop()
        shm_manager.shutdown()

if __name__ == "__main__":
    test_spacemouse()