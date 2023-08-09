from .ObTypes import *
from .Property import *
from .Error import ObException
#from .Pipeline import *
#from .StreamProfile import *
import StreamProfile
import Pipeline


import cv2
import numpy as np
import sys
from matplotlib import pyplot as plt

from primesense import openni2#, nite2
from primesense import _openni2 as c_api

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String

np.set_printoptions(threshold=sys.maxsize)

#dist = np.array([0.448018097, 3.60273275,  -0.000818341298, 0.00307180999, -1.49815960e+02])
#mtx = np.array([[2.34515245e+03, 0, 5.54018738e+02],[0, 2.36459745e+03, 4.52216370e+02],[0, 0, 1]])
q = 113
ESC = 27
alpha   = 0.3

pipe = Pipeline.Pipeline(None, None)

config = Pipeline.Config()

pipe.getDevice().setIntProperty(OB_PY_PROP_COLOR_POWER_LINE_FREQUENCY_INT, 60)

windowsWidth = 0
windowsHeight = 0

profiles = pipe.getStreamProfileList(OB_PY_SENSOR_COLOR)

videoProfile = None
videoProfile = profiles.getProfile(0)
colorProfile = videoProfile.toConcreteStreamProfile(OB_PY_STREAM_VIDEO)

config.enableStream(colorProfile)

profiles = pipe.getStreamProfileList(OB_PY_SENSOR_DEPTH)
videoProfile = None

videoProfile = profiles.getProfile(0)

depthProfile = videoProfile.toConcreteStreamProfile(OB_PY_STREAM_VIDEO)

config.enableStream(depthProfile)

config.setAlignMode(OB_PY_ALIGN_D2C_SW_MODE)

##CVSETTINGs

object_counter = 0

roi_top_left = (0, 0)  # Initialize the top-left coordinate of the ROI rectangle
roi_bottom_right = (0, 0)  # Initialize the bottom-right coordinate of the ROI rectangle

goal_object_contour = None
goal_object_size_mm = None
goal_object_centroid = None

pipe.start(config, None)
pipe.getDevice().setBoolProperty(OB_PY_PROP_DEPTH_MIRROR_BOOL, False)

new_origin_x = 329
new_origin_y = 0




def main(args=None):
   rclpy.init(args=args)
   node = rclpy.create_node('cpublisher')
   pub = node.create_publisher(String, 'Cobot_axles', 10)

   def mouse_callback(event, x, y, flags, param):
    global goal_object_contour, goal_object_size_mm, goal_object_centroid
    if event == cv2.EVENT_LBUTTONDOWN:
        goal_object_contour = None
        goal_object_size_mm = None
        goal_object_centroid = None
        for contour in contours:
            if cv2.pointPolygonTest(contour, (x, y), False) == 1:
                goal_object_contour = contour

                # Calculate the centroid of the reference object
                M = cv2.moments(goal_object_contour)
                goal_object_centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        depth_value = newDepthData[y, x]
        text = "Depth at ({}, {}): {}".format(x, y, depth_value)
        msg = String()
        msg.data = str(x) + ',' + str(y) + ',' + str(depth_value)
        pub.publish(msg)
        node.get_logger().info('publishing: "%s"' %msg.data)
        print(text)

   while True:
    frameSet = None
    colorFrame = None
    depthFrame = None
    key = cv2.waitKey(1)

    frameSet = pipe.waitForFrames(100)

    if frameSet == None:
      continue
    else:
      
      colorFrame = frameSet.colorFrame()
      depthFrame = frameSet.depthFrame()


      if colorFrame != None and depthFrame != None:
        colorSize = colorFrame.dataSize()
        colorData = colorFrame.data()
        depthSize = depthFrame.dataSize()
        depthData = depthFrame.data()
        colorWidth = colorFrame.width()
        colorHeight = colorFrame.height()
        colorFormat = colorFrame.format()
        depthWidth = depthFrame.width()
        depthHeight = depthFrame.height()
        valueScale = depthFrame.getValueScale()


        if colorData is not None and depthData is not None:
          newColorData = colorData
          newColorData = cv2.imdecode(newColorData,1)
          noiColorData = newColorData
          
#####################################
 

          

          if newColorData is not None:
            newColorData = np.resize(newColorData,(colorHeight, colorWidth, 3))
            #newColorData = cv2.undistort(newColorData, mtx, dist, None, mtx)
          depthData = np.resize(depthData,(depthHeight, depthWidth, 2))

          newDepthData = depthData[:,:,0]+depthData[:,:,1]*256   
          # Convert frame data to 1mm units
          newDepthData = (newDepthData * valueScale).astype('uint16')
          #newDepthData[newDepthData == 0] = 451
          #newDepthData[newDepthData > 450] = 450

          

          normalized_image = (newDepthData / 4).astype('uint8')

          
          normalized_image = cv2.equalizeHist(normalized_image)
          #depthedge=cv2.Canny(normalized_image, 100, 200)
          outputDepthImage = cv2.cvtColor(normalized_image, cv2.COLOR_GRAY2BGR)

          roi_top_left = (125, 25)  # Top-left coordinate of the ROI rectangle
          roi_bottom_right = (550, 435)  # Bottom-right coordinate of the ROI rectangle
          
##############################################
          gray = cv2.cvtColor(newColorData, cv2.COLOR_BGR2GRAY)
          _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
          roi = binary[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
          roi_blurred = cv2.GaussianBlur(roi, (3,3), 0)
          contours, _ = cv2.findContours(roi_blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

          if goal_object_contour is not None:
              cv2.drawContours(newColorData, [goal_object_contour], -1, (0, 0, 255), 2)
              cv2.circle(newColorData, goal_object_centroid, 5, (0, 0, 255), -1)

          

          for contour in contours:
            for point in contour:
                point[0][0] += roi_top_left[0]
                point[0][1] += roi_top_left[1]

          for contour in contours:
            if contour is not goal_object_contour and cv2.contourArea(contour) > 100 : 
                
                cv2.drawContours(newColorData, [contour], -1, (0, 255, 0), 1)
                
                M = cv2.moments(contour)
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                cv2.putText(newColorData, str(newDepthData[centroid_y, centroid_x]), (centroid_x, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                
                rect = cv2.minAreaRect(contour)

                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #cv2.drawContours(newColorData, [box], 0, (255, 255, 0), 1)

                _, (width, height), _ = rect
                object_length = max(width, height)
                object_length_text = f"{object_length:.2f} units"


                rect = cv2.minAreaRect(contour)
                angle = rect[-1]
                if angle < -45:
                    angle += 90

                box = cv2.boxPoints(rect)
                box = np.int0(box)
                center = tuple(np.mean(box, axis=0, dtype=int))
                line_length = int(object_length)
                angle_rad = np.radians(angle)  # Convert angle to radians
                end_point_x = center[0] + int(line_length * np.cos(angle_rad))
                end_point_y = center[1] + int(line_length * np.sin(angle_rad))
                color = (0, 255, 255)  # Yellow color
                thickness = 2
                cv2.line(newColorData, center, (end_point_x, end_point_y), color, thickness)

                # Display the rotated angle
                rotation_angle_deg = angle if angle > -45 else angle + 90
                angle_text = f"R: {rotation_angle_deg:.2f} '"
                cv2.putText(newColorData, angle_text, (center[0], center[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                
                new_center_x = center[0] - new_origin_x
                new_center_y = center[1] - new_origin_y  # Invert y-coordinate
                new_end_point_x = end_point_x - new_origin_x
                new_end_point_y = end_point_y - new_origin_y

                new_coordinate_text = f"({new_center_x}, {new_center_y})"
                cv2.putText(newColorData, new_coordinate_text, (center[0], center[1] + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)


                # Display the object's coordinates
                #coordinate_text = f"X: {center[0]}, Y: {center[1]}"
                #cv2.putText(newColorData, coordinate_text, (center[0], center[1] - 10),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)


                
                x = centroid_x - int(object_length / 2)
                y = centroid_y + 20

          #cv2.drawContours(newColorData, contours, -1, (0, 255, 0), 1)
          cv2.rectangle(newColorData, roi_top_left, roi_bottom_right, (0, 255, 0), 2)

          
          #############################################
          
          if colorHeight != depthHeight:
            outputDepthImage = cv2.resize(outputDepthImage,(colorWidth,colorHeight))

          if newColorData is not None:
            newData = newColorData
          if outputDepthImage is not None:
            newData = outputDepthImage
          if newColorData is not None and outputDepthImage is not None:
            newData = cv2.addWeighted(newColorData, (1 - alpha), outputDepthImage, alpha, 0)
            zero_depth_mask = (newDepthData == 0)
            outputDepthImage[zero_depth_mask] = [0, 0, 255]
        
          
          
          
          cv2.namedWindow("xyzviewer", cv2.WINDOW_NORMAL)
          cv2.setMouseCallback("xyzviewer", mouse_callback)  
          
          cv2.imshow("xyzviewer", newColorData)
          #cv2.imshow("xyzviewer", outputDepthImage)
          
          if key == ESC or key == q:
            cv2.destroyAllWindows()
            break
   

if __name__=='__main__':
    main()
    pipe.stop()
