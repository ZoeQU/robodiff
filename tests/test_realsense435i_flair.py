#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense D435i/D435 example             ##
#####################################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time
 
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
# Start streaming
pipeline.start(config)

# initial encoder
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out_color = cv2.VideoWriter('data/test/435i_output_color.mp4', fourcc, 30.0, (640, 480))
out_depth = cv2.VideoWriter('data/test/435i_output_depth.mp4', fourcc, 30.0, (640, 480))

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
 
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
 
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
 
        # # Stack both images horizontally
        # images = np.hstack((color_image, depth_colormap))
 
        # # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)

        # write video
        out_color.write(color_image)
        out_color.write(depth_colormap)
        
        # Show images
        cv2.imshow('RealSense Color', color_image)
        cv2.imshow('RealSense Depth', depth_colormap)
 
 
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
 
finally:
 
    # Stop streaming
    pipeline.stop()
    out_color.release()
    out_depth.release()
    cv2.destroyAllWindows()