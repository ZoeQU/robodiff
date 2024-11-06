"""wrote by zoe qu"""
from typing import Dict
import torch
import numpy as np
import copy

import zarr
from matplotlib import pyplot as plt
import matplotlib.cm as cm
import os
import cv2


class DatasetAnalyzer:
    ##### read only ####
    def __init__(self, file_path, n):
        self.zarr_file = zarr.open(file_path, mode="r")
        self.n = n

    def print_structure(self, group=None, indent=0):
        if group is None:
            group = self.zarr_file
        for key, item in group.items(): # type:ignore
            print(" " * indent + key)
            if isinstance(item, zarr.Group):
                self.print_structure(item, indent + 2)

    def recursive_visualize(self, group=None, path=""):
        if group is None:
            group = self.zarr_file
        for key, item in group.items(): # type:ignore
            current_path = f"{path}/{key}" if path else key
            if isinstance(item, zarr.Group):
                print(f"Group: {current_path}")
                self.recursive_visualize(item, current_path)
            elif isinstance(item, zarr.Array):
                print(f"Array: {current_path}")
                if "img" in current_path:
                    self.visualize_image(item, current_path)
                if "action" in current_path:
                    self.read_action_data(item, self.n)
                if "keypoint" in current_path:
                    self.read_keypoint_data(item, self.n)
                if "n_contacts" in current_path:
                    self.read_n_contacts_data(item, self.n)
                if "state" in current_path:
                    self.read_state_data(item, self.n)
                if "robot_eef_pose" in current_path and "robot_eef_pose_vel" not in current_path:
                    self.read_robot_eef_pose(item, self.n)
                if "robot_eef_pose_vel" in current_path:
                    self.read_robot_eef_pose_vel(item, self.n)
                if "robot_joint" in current_path and "robot_joint_vel" not in current_path:
                    self.read_robot_joint(item, self.n)
                if "robot_joint_vel" in current_path:
                    self.read_robot_joint_vel(item, self.n)
                if "stage" in current_path:
                    self.read_stage(item, self.n)
                if "timestamp" in current_path:
                    self.read_timestamp(item, self.n)
                if "episode_ends" in current_path:
                    self.read_meta_episode_ends(item, self.n)
 
    def read_meta_episode_ends(self, array, n):
        episode_ends_data = array[:n]
        print("Episode Ends Data:", episode_ends_data)

    def visualize_image(self, array, title):
        img_data = array[:]
        # print("img_data.shape: ", img_data.shape)     #(25650, 96, 96, 3)
        for i, img in enumerate(img_data):        
            if i % 100 == 0: 
                # cv2.imwrite(f"data_real_img_{i}.png", img)
                pass
                
    def read_action_data(self, array, n):
        action_data = array[:]
        # print("action_data: ", array.shape) #sim: (25650, 2) real:(27672, 6)
        print("action: ", action_data[:n])
        
    def read_keypoint_data(self, array, n):
        keypoint_data = array[:]
        # print("keypoint_data.shape: ", keypoint_data.shape) # (25650, 9, 2)
        print("keypoint_data: ", keypoint_data[:n])

        for i in range(n):
            keypoint = np.array(keypoint_data[i])
            x = keypoint[:,0]
            y = keypoint[:,1]

            plt.scatter(x, y)
            for j in range(len(x)):
                plt.text(x[j], y[j], str(j), fontsize=9, ha="right")

            # fixed color board
            fixed_colors = ['red', 'green', 'blue', 'orange', 'purple', 'brown', 'pink', 'gray', 'cyan', 'magenta']
            # random color board
            colors = cm.rainbow(np.linspace(0, 1, len(x) - 1))  # type:ignore
            for j in range(len(x) - 1):
                plt.plot(x[j:j+2], y[j:j+2], color=fixed_colors[j])

            plt.xlabel('x')
            plt.ylabel('y')
            plt.xlim(0, 500)
            plt.ylim(0, 500)
        
            plt.title(f'Keypoints_{i}')
            plt.show()
            # plt.savefig("keypoints_fig.png")
            # plt.close()
        
    def read_n_contacts_data(self, array, n):
        read_n_contacts_data = array[:]
        # print("read_n_contacts_data: ", read_n_contacts_data.shape) #(25650, 1)
        print("read_n_contacts_data: ", read_n_contacts_data[0:n])

    def read_state_data(self, array, n):
        state_data = array[:]
        # print("state_data.shape: ", state_data.shape) #(25650, 5)
        print("state_data: ", state_data[:n])

    def read_robot_eef_pose(self, array, n):
        # real: (25672, 6)
        eef_pose = array[:]
        print("robot_ee_pose: ", eef_pose[:n])

    def read_robot_eef_pose_vel(self, array, n):
        eef_pose_vel = array[:]
        # real: (25672, 6)
        print("robot_eef_pose_vel: ", eef_pose_vel[:n])

    def read_robot_joint(self, array, n):
        robot_joint = array[:]
        # real: (25672, 6)
        print("robot_joint: ", robot_joint[:n])

    def read_robot_joint_vel(self, array, n):
        robot_joint_vel = array[:]
        # real: (25672, 6)
        print("robot_joint_vel: ", robot_joint_vel[:n])

    def read_stage(self, array, n):
        # real: (25672)
        stage = array[:]
        print("stage: ", stage[:n]) # 0, 1, 2

    def read_timestamp(self, array, n):
        # real: (25672)
        timestamp = array[:]
        print("timestamp: ", timestamp[:n])




if __name__ == "__main__":
    # path = "./data/pusht_real/real_pusht_20230105/replay_buffer.zarr"
    # path = "./data/pusht/pusht_cchi_v7_replay.zarr"
    path = "./data/demo_pusht_real/replay_buffer.zarr"
    analyzer = DatasetAnalyzer(path, 6)
    # print("Zarr structure:")
    # analyzer.print_structure()
    # print("\nData sturcture visualization:")
    analyzer.recursive_visualize()
    