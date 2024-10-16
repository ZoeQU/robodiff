# region

# import evdev
# from evdev import InputDevice, categorize, ecodes

# # 替换为你的鼠标设备路径
# device_path = '/dev/input/event19'
# mouse = InputDevice(device_path)

# print(f'监听设备: {mouse}')

# # 按键映射
# button_map = {
#     ecodes.BTN_LEFT: '左键',
#     ecodes.BTN_RIGHT: '右键',
#     ecodes.BTN_MIDDLE: '中键',
#     ecodes.BTN_SIDE: '侧键1',
#     ecodes.BTN_EXTRA: '侧键2',
#     ecodes.BTN_FORWARD: '侧键3',
#     ecodes.BTN_BACK: '侧键4'
# }

# for event in mouse.read_loop():
#     if event.type == ecodes.EV_KEY:
#         key_event = categorize(event)
#         if key_event.keystate == key_event.key_down:
#             print(f'{button_map.get(key_event.scancode, "未知按键")} 按下')
#         elif key_event.keystate == key_event.key_up:
#             print(f'{button_map.get(key_event.scancode, "未知按键")} 释放')
#     elif event.type == ecodes.EV_REL:
#         if event.code == ecodes.REL_X:
#             print(f'鼠标水平移动: {event.value}')
#         elif event.code == ecodes.REL_Y:
#             print(f'鼠标垂直移动: {event.value}')
#         elif event.code == ecodes.REL_WHEEL:
#             print(f'鼠标滚轮 {"向上" if event.value > 0 else "向下"} 滚动')

#######################################################################################################################
#######################################################################################################################

# import evdev
# from evdev import InputDevice, categorize, ecodes
# import threading
# import time

# # 替换为你的鼠标设备路径
# device_path = '/dev/input/event19'
# mouse = InputDevice(device_path)

# print(f'监听设备: {mouse}')

# # 按键映射
# button_map = {
#     ecodes.BTN_LEFT: '左键',
#     ecodes.BTN_RIGHT: '右键',
#     ecodes.BTN_MIDDLE: '中键',
#     ecodes.BTN_SIDE: '侧键1',
#     ecodes.BTN_EXTRA: '侧键2',
#     ecodes.BTN_FORWARD: '侧键3',
#     ecodes.BTN_BACK: '侧键4'
# }

# # 状态变量
# motion_state = {
#     'x': 0,
#     'y': 0,
#     'z': 0,
#     'rot_x': 0,
#     'rot_y': 0,
#     'rot_z': 0
# }

# def read_mouse_events():
#     for event in mouse.read_loop():
#         if event.type == ecodes.EV_KEY:
#             key_event = categorize(event)
#             if key_event.keystate == key_event.key_down:
#                 if key_event.scancode == ecodes.BTN_LEFT:
#                     motion_state['z'] += 1
#                 elif key_event.scancode == ecodes.BTN_RIGHT:
#                     motion_state['z'] -= 1
#                 elif key_event.scancode == ecodes.BTN_SIDE:
#                     motion_state['rot_x'] += 1
#                 elif key_event.scancode == ecodes.BTN_EXTRA:
#                     motion_state['rot_x'] -= 1
#                 elif key_event.scancode == ecodes.BTN_FORWARD:
#                     motion_state['rot_y'] += 1
#                 elif key_event.scancode == ecodes.BTN_BACK:
#                     motion_state['rot_y'] -= 1
#                 print(f'{button_map.get(key_event.scancode, "未知按键")} 按下')
#             elif key_event.keystate == key_event.key_up:
#                 print(f'{button_map.get(key_event.scancode, "未知按键")} 释放')
#         elif event.type == ecodes.EV_REL:
#             if event.code == ecodes.REL_X:
#                 motion_state['x'] += event.value
#                 print(f'鼠标水平移动: {event.value}')
#             elif event.code == ecodes.REL_Y:
#                 motion_state['y'] += event.value
#                 print(f'鼠标垂直移动: {event.value}')
#             elif event.code == ecodes.REL_WHEEL:
#                 motion_state['rot_z'] += event.value
#                 print(f'鼠标滚轮 {"向上" if event.value > 0 else "向下"} 滚动')

# def print_motion_state():
#     while True:
#         print(f'当前状态: {motion_state}')
#         time.sleep(1)

# # 启动线程读取鼠标事件
# mouse_thread = threading.Thread(target=read_mouse_events)
# mouse_thread.daemon = True
# mouse_thread.start()

# # 启动线程打印状态
# print_thread = threading.Thread(target=print_motion_state)
# print_thread.daemon = True
# print_thread.start()

# # 保持主线程运行
# try:
#     while True:
#         time.sleep(1)
# except KeyboardInterrupt:
#     print("程序终止")

#######################################################################################################################
#######################################################################################################################
# endregion


import evdev
from evdev import InputDevice, categorize, ecodes
import threading
import time
"""in this version, all common 2D mouse is OK"""
# sudo evtest #find which device
# (if meet error: permission denied) sudo chmod 666 /dev/input/event7
device_path = '/dev/input/event7' 
mouse = InputDevice(device_path)

print(f'Listening to: {mouse.name}')

# 按键映射
button_map = {
    ecodes.BTN_LEFT: 'Left Key',
    ecodes.BTN_RIGHT: 'Right Key',
    ecodes.BTN_MIDDLE: 'Middle Key',
    ecodes.BTN_SIDE: 'Side 1',
    ecodes.BTN_EXTRA: 'Side 2',
}

# region
# for event in mouse.read_loop():
#     if event.type == ecodes.EV_KEY:
#         key_event = categorize(event)
#         button_name = button_map.get(key_event.scancode, "unkown_button")

#         if key_event.keystate == key_event.key_down:
#             print(f"{button_name} is clicked")
#         elif key_event.keystate == key_event.key_up:
#             print(f"{button_name} is released")
# endregion


# 状态变量
motion_state = {
    'x': 0,
    'y': 0,
    'z': 0,
    'rot_x': 0,
    'rot_y': 0,
    'rot_z': 0
}

# 模式切换变量
is_rotation_mode = False

def read_mouse_events():
    global is_rotation_mode
    for event in mouse.read_loop():
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            if key_event.scancode == ecodes.BTN_SIDE and key_event.keystate == key_event.key_down:  # click side1 for mode change
                is_rotation_mode = not is_rotation_mode
                mode = "旋转模式" if is_rotation_mode else "平移模式"
                print(f'切换到 {mode}')
            if key_event.keystate == key_event.key_down:
                print(f'{button_map.get(key_event.scancode, "未知按键")} 按下')
            elif key_event.keystate == key_event.key_up:
                print(f'{button_map.get(key_event.scancode, "未知按键")} 释放')
        elif event.type == ecodes.EV_REL:
            if is_rotation_mode:
                if event.code == ecodes.REL_X:
                    motion_state['rot_x'] += event.value
                    # print(f'旋转 x 轴: {event.value}')
                elif event.code == ecodes.REL_Y:
                    motion_state['rot_y'] += event.value
                    # print(f'旋转 y 轴: {event.value}')
                elif event.code == ecodes.REL_WHEEL:
                    motion_state['rot_z'] += event.value
                    # print(f'旋转 z 轴: {event.value}')
            else:
                if event.code == ecodes.REL_X:
                    motion_state['x'] += event.value
                    # print(f'平移 x 轴: {event.value}')
                elif event.code == ecodes.REL_Y:
                    motion_state['y'] += event.value
                    # print(f'平移 y 轴: {event.value}')
                elif event.code == ecodes.REL_WHEEL:
                    motion_state['z'] += event.value
                    # print(f'平移 z 轴: {event.value}')

def print_motion_state():
    while True:
        print(f'当前状态: {motion_state}')
        time.sleep(1)

# 启动线程读取鼠标事件
mouse_thread = threading.Thread(target=read_mouse_events)
mouse_thread.daemon = True
mouse_thread.start()

# 启动线程打印状态
print_thread = threading.Thread(target=print_motion_state)
print_thread.daemon = True
print_thread.start()

# 保持主线程运行
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("程序终止")
