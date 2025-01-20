import test_abb as abb
import time
import os

R = abb.Robot(ip='192.168.125.108',port_motion=5000) # simulation
# R = abb.Robot(ip='192.168.125.1',port_motion=5000) # real robot
print("connect success")
while True:
    a4 = R.get_cartesian()
    print("Robot at ", a4)
    a3 = list(map(float, input("input delta xyz, format as 1 1 1\n").split()))
    # print("you input delta xyz ", a3)
    # a4 = [[400.0, -112.0, 594.0], [0.5, -0.0, 0.866, 0.0]]
    a4[0][0]=  a4[0][0]+  a3[0]
    a4[0][1] = a4[0][1] + a3[1]
    a4[0][2] = a4[0][2] + a3[2]
    R.set_cartesian(a4)

    print(f"joints: {R.get_joints()}")
    # a3 = int(input("year\n"))
    # a4=R.get_cartesian()
    # print(a4)
    time.sleep(1)