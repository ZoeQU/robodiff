import multiprocessing as mp
import numpy as np
import time
from multiprocessing.managers import SharedMemoryManager
from pynput import mouse
from diffusion_policy.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
import evdev
from evdev import InputDevice, categorize, ecodes
import threading


class MouseController(mp.Process):
    def __init__(self, 
                 shm_manager, 
                 get_max_k=30, 
                 frequency=200, 
                 max_value=500, 
                 deadzone=(0, 0, 0, 0, 0, 0), 
                 dtype=np.float32, 
                 n_buttons=6,
                 device_path='/dev/input/event7'): # Path for evdev mouse # sudo evtest #find which device # (if meet error: permission denied) sudo chmod 666 /dev/input/event7
        super().__init__()
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        # copied variables
        self.frequency = frequency
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.n_buttons = n_buttons
        self.mode = 'translate'  # default mode
        self.last_position = (0, 0)  

        self.scale_factor = 20 # Scale factor for movement (could be removed later)

        example = {
            # 3 translation, 3 rotation, 1 period
            'motion_event': np.zeros((7,), dtype=np.int64),
            # left and right button
            'button_state': np.zeros((n_buttons,), dtype=bool),
            'receive_timestamp': time.time()
        }
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        self.stop_event = mp.Event()
        self.ring_buffer = ring_buffer

        # Evdev mouse setup
        self.device_path = device_path
        self.evdev_mouse = InputDevice(self.device_path)
        self.is_rotation_mode = False
        self.motion_state = {
            'x': 0,
            'y': 0,
            'z': 0,
            'rot_x': 0,
            'rot_y': 0,
            'rot_z': 0
        }


    def get_motion_state(self):
        state = self.ring_buffer.get()
        state = np.array(state['motion_event'][:6], dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state

    def get_button_state(self):
        state = self.ring_buffer.get()
        return state['button_state']

    def is_button_pressed(self, button_id):
        return self.get_button_state()[button_id]
    
    #========== start stop API ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.ready_event.wait()

    def stop(self, wait=True):
        self.stop_event.set()
        if wait:
            self.join()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # def on_move(self, x, y):
    #     if self.mode == 'translate':
    #         dx = x - self.last_position[0]
    #         dy = y - self.last_position[1]
    #         motion_event = np.array([dx, dy, 0, 0, 0, 0, 0], dtype=np.int64)
    #     else:  # 'rotate'
    #         dx = x - self.last_position[0]
    #         dy = y - self.last_position[1]
    #         motion_event = np.array([0, 0, 0, dx, dy, 0, 0], dtype=np.int64)

    #     self.ring_buffer.put({
    #         'motion_event': motion_event,
    #         'button_state': [False] * self.n_buttons,
    #         'receive_timestamp': time.time()
    #     })
    #     self.last_position = (x, y)


    def on_move(self, x, y):
        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        
        # Apply scaling factor to movement
        motion_event = np.array([
            dx * self.scale_factor,
            dy * self.scale_factor,
            0, 0, 0, 0, 0
        ], dtype=np.int64)

        if self.mode == 'rotate':
            motion_event = np.array([0, 0, 0, dx * self.scale_factor, dy * self.scale_factor, 0, 0], dtype=np.int64)

        self.ring_buffer.put({
            'motion_event': motion_event,   # type: ignore
            'button_state': [False] * self.n_buttons,
            'receive_timestamp': time.time()
        })
        self.last_position = (x, y)

    def on_click(self, x, y, button, pressed):
        if button == mouse.Button.left:
            if pressed:
                self.mode = 'rotate' if self.mode == 'translate' else 'translate'

    # def on_scroll(self, x, y, dx, dy):
    #     z_motion = np.array([0, 0, dy, 0, 0, 0, 0], dtype=np.int64)
    #     if self.mode == 'rotate':
    #         z_motion[4] = dy  # handle Z axix rotation
    #     self.ring_buffer.put({
    #         'motion_event': z_motion,
    #         'button_state': [False] * self.n_buttons,
    #         'receive_timestamp': time.time()
    #     })

    def on_scroll(self, x, y, dx, dy):
        z_motion = np.array([0, 0, dy * self.scale_factor, 0, 0, 0, 0], dtype=np.int64)
        if self.mode == 'rotate':
            z_motion[4] = dy * self.scale_factor  # Handle Z axis rotation
        self.ring_buffer.put({
            'motion_event': z_motion,   # type: ignore
            'button_state': [False] * self.n_buttons,
            'receive_timestamp': time.time()
        })


    def read_evdev_mouse_events(self):
        for event in self.evdev_mouse.read_loop():
            if event.type == ecodes.EV_KEY: # type: ignore
                key_event = categorize(event)
                if key_event.scancode == ecodes.BTN_LEFT and key_event.keystate == key_event.key_down:  # type: ignore
                    self.is_rotation_mode = not self.is_rotation_mode
                    mode = "rotate" if self.is_rotation_mode else "translate"
                    print(f'change to mode: {mode}')
            elif event.type == ecodes.EV_REL:   # type: ignore
                if self.is_rotation_mode:
                    if event.code == ecodes.REL_X:  # type: ignore
                        self.motion_state['rot_x'] += event.value
                        print(f'rotate x axis: {event.value}')
                    elif event.code == ecodes.REL_Y:    # type: ignore
                        self.motion_state['rot_y'] += event.value
                        print(f'rotate y axis: {event.value}')
                    elif event.code == ecodes.REL_WHEEL:    # type: ignore
                        self.motion_state['rot_z'] += event.value
                        print(f'rotate z axis: {event.value}')
                else:
                    if event.code == ecodes.REL_X:  # type: ignore
                        self.motion_state['x'] += event.value
                        print(f'translate x axis: {event.value}')
                    elif event.code == ecodes.REL_Y:    # type: ignore
                        self.motion_state['y'] += event.value
                        print(f'translate y axis: {event.value}')
                    elif event.code == ecodes.REL_WHEEL:    # type: ignore
                        self.motion_state['z'] += event.value
                        print(f'translate z axis: {event.value}')

    # ========= main loop ==========
    def run(self):
        self.ready_event.set()
        
        # Create the listener
        listener = mouse.Listener(
            on_move=self.on_move,
            on_click=self.on_click,
            on_scroll=self.on_scroll
        )
        
        listener.start()
        
        # Start reading evdev mouse events in a separate thread
        evdev_thread = threading.Thread(target=self.read_evdev_mouse_events)
        evdev_thread.daemon = True
        evdev_thread.start()

        # Keep the process running until stop_event is set
        while not self.stop_event.is_set():
            time.sleep(0.1)  # Sleep briefly to avoid busy-waiting
        
        # Stop the listener when the stop event is triggered
        listener.stop()
        listener.join()  # Ensure the listener thread has finished


