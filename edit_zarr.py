import zarr
import numpy as np

z = zarr.open("data/demo_pusht_real/replay_buffer.zarr", mode='r+')

# Define indices to keep (excluding those you want to remove)
indices_to_remove = [31, 39, 125]
indices_to_keep = np.setdiff1d(np.arange(len(z['data/action'])), indices_to_remove)

new_z = zarr.open("data/demo_pusht_real/clean_replay_buffer.zarr", mode='w')

# Copy the selected data to the new Zarr file
subfolders = [
    'data/action',
    'data/robot_eef_pose',
    'data/robot_eef_pose_vel',
    'data/robot_joint',
    'data/robot_joint_vel',
    'data/stage',
    'data/timestamp'
]

for subfolder in subfolders:
    data = z[subfolder][:]
    new_z[subfolder] = data[indices_to_keep]
    
# # print(len(z['data/action']))
# # print(len(z['meta/episode_ends']))
# # Define the subfolders and indices to remove
# subfolders = [
#     'data/action',
#     'data/robot_eef_pose',
#     'data/robot_eef_pose_vel',
#     'data/robot_joint',
#     'data/robot_joint_vel',
#     'data/stage',
#     'data/timestamp'
# ]
# indices_to_remove = [31, 39, 125]

# # Remove the specified indices
# for subfolder in subfolders:
#     data = z[subfolder][:]
#     mask = np.ones(data.shape[0], dtype=bool)
#     mask[indices_to_remove] = False
#     # Create a new dataset with the filtered data
#     z[subfolder] = data[mask]

# # Update episode_ends by removing specified indices
# episode_ends = z['meta/episode_ends'][:]
# episode_ends = np.delete(episode_ends, indices_to_remove)
# z['meta/episode_ends'][:] = episode_ends

# print("biu~biu~biu~")



