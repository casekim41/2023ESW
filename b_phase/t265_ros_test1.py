#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function

import pyrealsense2 as rs

import cv2
import numpy as np
from math import tan, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String



def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

from threading import Lock
frame_mutex = Lock()
frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None
              }

def callback(frame):
    global frame_data
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        frame_mutex.acquire()
        frame_data["left"] = left_data
        frame_data["right"] = right_data
        frame_data["timestamp_ms"] = ts
        frame_mutex.release()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and stream everything
cfg = rs.config()
# Start streaming with our callback
profile = pipe.start(cfg)
pipe.stop()
sensor = profile.get_device().query_sensors()[0] 

max_exp = sensor.get_option_range(rs.option.exposure).max
min_exp = sensor.get_option_range(rs.option.exposure).min



sensor.set_option(rs.option.enable_auto_exposure, 0)
sensor.set_option(rs.option.exposure, 4000)

print(sensor.get_option(rs.option.exposure))

pipe.start(cfg, callback)




def main(args=None):
    rclpy.init(args=args)
    mode='stack'

    window_size = 5
    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                   numDisparities = num_disp,
                                   blockSize = 16,
                                   P1 = 8*3*window_size**2,
                                   P2 = 32*3*window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32)
    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()
    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}
    # Print information about both cameras
    print("Left camera:",  intrinsics["left"])
    print("Right camera:", intrinsics["right"])
    # Translate the intrinsics from librealsense into OpenCV
    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)
    # Get the relative extrinsics between the left and right camera
    (R, T) = get_extrinsics(streams["left"], streams["right"])

    stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
    stereo_height_px = 400          # 300x300 pixel stereo output
    stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)
    # We set the left rotation to identity and the right rotation
    # the rotation between the cameras
    R_left = np.eye(3)
    R_right = R
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0,               0,         1, 0]])
    P_right = P_left.copy()
    P_right[0][3] = T[0]*stereo_focal_px
    # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
    # since we will crop the disparity later
    Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                  [0, 1,       0, -stereo_cy],
                  [0, 0,       0, stereo_focal_px],
                  [0, 0, -1/T[0], 0]])
    # Create an undistortion map for the left and right camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left"  : (lm1, lm2),
                         "right" : (rm1, rm2)}
    mode = "stack"
    roi_pts = np.array([(70,60), (480, 110), (450, 330), (40, 280)], np.int32)  # Define the vertices of the trapezoid

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('coordpub')
    pub = node.create_publisher(String, 'depthcoord', qos)

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            msg = String()
            depth_value = disparity[y, x]
            print("Depth at ({}, {}): {}".format(x, y, depth_value))
            msg.data = str(x) + ','+ str(y) +',' + str(depth_value)
            pub.publish(msg)
            node.get_logger().info('Publishing: "%s"' % msg.data)

    
    try:
        while True:
            WINDOW_TITLE = 'Realsense'
            cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(WINDOW_TITLE, mouse_callback)    

            # Check if the camera has acquired any frames
            frame_mutex.acquire()
            valid = frame_data["timestamp_ms"] is not None
            frame_mutex.release()

            # If frames are ready to process
            if valid:
                # Hold the mutex only long enough to copy the stereo frames
                frame_mutex.acquire()
                frame_copy = {"left"  : frame_data["left"].copy(),
                              "right" : frame_data["right"].copy()}
                frame_mutex.release()

                # Undistort and crop the center of the frames
                center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                              map1 = undistort_rectify["left"][0],
                                              map2 = undistort_rectify["left"][1],
                                              interpolation = cv2.INTER_LINEAR),
                                      "right" : cv2.remap(src = frame_copy["right"],
                                              map1 = undistort_rectify["right"][0],
                                              map2 = undistort_rectify["right"][1],
                                              interpolation = cv2.INTER_LINEAR)}

                # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
                disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0
                d=stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0


                # re-crop just the valid part of the disparity
                disparity = disparity[:,max_disp:]


                #roi_mask = np.zeros_like(disparity, dtype=np.uint8)
                roi_mask = np.zeros_like(center_undistorted["left"], dtype=np.uint8)
                cv2.fillPoly(roi_mask, [roi_pts], 255)  # Fill the trapezoid shape with white pixels (255)

                #roi_mask = roi_mask[0:disparity.shape[0], 0:disparity.shape[1]]
                roi_mask = cv2.resize(roi_mask, (disparity.shape[1], disparity.shape[0]))




                invalid_mask = np.logical_and(disparity == -1, roi_mask)

                disparity = cv2.inpaint(disparity, invalid_mask.astype(np.uint8), 2, cv2.INPAINT_NS)


                roi_mask = roi_mask.astype(disparity.dtype)
                masked_disparity = cv2.bitwise_and(disparity, roi_mask)

                blur_kernel_size = 5
                blur_sigma = 2.0


                disparity_blurred = cv2.GaussianBlur(masked_disparity, (blur_kernel_size, blur_kernel_size), blur_sigma)
                disp_vis = 255 * (disparity_blurred - min_disp) / num_disp

                disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis, alpha=1), cv2.COLORMAP_JET)



                color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)

                if mode == "stack":
                    cv2.imshow(WINDOW_TITLE, np.hstack((color_image, disp_color)))
                if mode == "overlay":
                    ind = disparity >= min_disp
                    color_image[ind, 0] = disp_color[ind, 0]
                    color_image[ind, 1] = disp_color[ind, 1]
                    color_image[ind, 2] = disp_color[ind, 2]
                    cv2.imshow(WINDOW_TITLE, color_image)
            key = cv2.waitKey(1)
        
            if key == ord('s'): mode = "stack"
            if key == ord('o'): mode = "overlay"
            if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break

            #rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pipe.stop()
