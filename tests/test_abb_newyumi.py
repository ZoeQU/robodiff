# test.py
import sys

project_path = "/home/flair/Projects/robodiff/"
if project_path not in sys.path:
    sys.path.insert(0, project_path)

import time
from diffusion_policy.real_world.abb import Robot

def test_robot():
    # 初始化 Robot 实例
    robot_ip = "192.168.125.108"  # 替换为实际的机械臂 IP
    port_motion = 5000
    port_logger = 5002
    print("[TEST] Connecting to ABB robot...")
    robot = Robot(ip=robot_ip, port_motion=port_motion, port_logger=port_logger)

    # 测试 get_robotinfo
    print("[TEST] Getting robot info...")
    robot_info = robot.get_robotinfo()
    print("Robot Info:", robot_info)

    # 测试 set_units
    print("[TEST] Setting units...")
    robot.set_units("millimeters", "degrees")

    # # 测试 接收频率
    duration = 10
    print("[RECEIVER] Receiving messages for 1 minutes...")
    start_time = time.time()
    message_count = 0

    while time.time() - start_time < duration:
        tcp_pose = robot.getActualTCPPose()
        joints = robot.getActualQ()
        # print("tcp_pose: ", tcp_pose)
        # print("joints: ", joints)
        if tcp_pose != [0,0,0,0,0,0]: #and joints != [0,0,0,0,0,0,0]:
            # print("tcp_pose: ", tcp_pose)
            # print("joints", joints)
            message_count += 1

    elapsed_time = time.time() - start_time
    messages_per_second = message_count / elapsed_time
    print("elapsed_time: " , elapsed_time)
    print("messages_per_second: ", str(messages_per_second))

    

    # # 测试 set_cartesian
    # # print("[TEST] Moving to a Cartesian position...")
    # # # test_pose = [[14.9, 208.5 + i, 192.2], [0.03, 0.822, -0.141, 0.552]]  # 目标 TCP 位姿
    
    # test_pose = [-9.58, 307.89, 228.77, -179.34, -63.63, -15.45]  # 目标 TCP 位姿 ez ey ex
    # robot.set_cartesian(test_pose)
    # cartesian_pose = robot.get_cartesian_t2() # EX, EY, EZ
    # print("Current Cartesian Pose:", cartesian_pose)
    # time.sleep(2)


    # # 测试 get_cartesian
    # print("[TEST] Getting current Cartesian position...")
    # cartesian_pose = robot.get_cartesian()
    # print("Current Cartesian Pose:", cartesian_pose)

    # # 测试 set_joints
    # print("[TEST] Moving to a joint position...")
    # test_joints = [0, -130, 35, 0, 40, 0, 135]  # 目标关节角度（单位为度）
    # robot.set_joints(test_joints)
    # time.sleep(2)

    # # 测试 get_joints
    # print("[TEST] Getting current joint positions...")
    # joint_positions = robot.get_joints()
    # print("Current Joint Positions:", joint_positions)

    # # 测试 get_cartesian_t2
    # print("[TEST] Getting latest Cartesian pose from queue...")
    # cartesian_t2 = robot.get_cartesian_t2()
    # print("Latest Cartesian Pose (t2):", cartesian_t2)

    # # 测试 get_joints_t2
    # print("[TEST] Getting latest joint positions from queue...")
    # joints_t2 = robot.get_joints_t2()
    # print("Latest Joint Positions (t2):", joints_t2)

    # # 测试 set_tool
    # print("[TEST] Setting tool...")
    # test_tool = [[0, 0, 100], [1, 0, 0, 0]]  # 工具偏移
    # robot.set_tool(test_tool)

    # # 测试 get_tool
    # print("[TEST] Getting tool...")
    # tool = robot.get_tool()
    # print("Current Tool:", tool)

    # # 测试 set_workobject
    # print("[TEST] Setting workobject...")
    # test_workobject = [[0, 0, 0], [1, 0, 0, 0]]  # 工件坐标系
    # robot.set_workobject(test_workobject)

    # # 测试 set_speed
    # print("[TEST] Setting speed...")
    # test_speed = [100, 50, 50, 50]  # TCP 速度和外部轴速度
    # robot.set_speed(test_speed)

    # # 测试 set_zone
    # print("[TEST] Setting motion zone...")
    # robot.set_zone("z10")

    # # 测试 initPeriod 和 waitPeriod
    # print("[TEST] Testing initPeriod and waitPeriod...")
    # t_start = robot.initPeriod()
    # print("Start Time:", t_start)
    # robot.waitPeriod(t_start)
    # print("[TEST] Period waiting done!")

    # # 测试 getActualTCPPose
    # print("[TEST] Getting actual TCP pose...")
    # actual_tcp_pose = robot.getActualTCPPose()
    # print("Actual TCP Pose:", actual_tcp_pose)

    # # 测试 getActualQ
    # print("[TEST] Getting actual joint positions...")
    # actual_joint_positions = robot.getActualQ()
    # print("Actual Joint Positions:", actual_joint_positions)

    # # 测试 buffer_set 和 buffer_execute
    # print("[TEST] Testing buffer_set and buffer_execute...")
    # buffer_poses = [
    #     [[13.9, 208.5, 180.2], [0.03, 0.822, -0.141, 0.552]],
    #     [[14.9, 218.5, 186.2], [0.03, 0.822, -0.141, 0.552]],
    #     [[15.9, 228.5, 192.2], [0.03, 0.822, -0.141, 0.552]]
    # ]
    # robot.buffer_set(buffer_poses)
    # print("Executing buffer...")
    # robot.buffer_execute()

    # # 测试 servoL
    # print("[TEST] Testing servoL...")
    # pose_command = [-11.42, 281.45, 198.63, -179.34, -63.63, -15.45]  # 目标位姿 [毫米, 四元数]
    # robot.servoL(pose_command, vel=0.5, acc=0.2, dt=0.1, lookahead_time=0.1, gain=300)


    # 关闭连接
    robot.close()
    print("[TEST] Robot connection closed.")


if __name__ == "__main__":
    test_robot()