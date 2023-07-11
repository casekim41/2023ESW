from ObTypes import *
from Property import *
import Pipeline
import StreamProfile
from Error import ObException
import cv2
import numpy as np
import sys
from matplotlib import pyplot as plt

from primesense import openni2#, nite2
from primesense import _openni2 as c_api

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

pipe.start(config, None)
pipe.getDevice().setBoolProperty(OB_PY_PROP_DEPTH_MIRROR_BOOL, False)


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_value = newDepthData[y, x]
        text = "Depth at ({}, {}): {}".format(x, y, depth_value)
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
          
#####################################3
 

          

          if newColorData is not None:
            newColorData = np.resize(newColorData,(colorHeight, colorWidth, 3))
            #newColorData = cv2.undistort(newColorData, mtx, dist, None, mtx)
          depthData = np.resize(depthData,(depthHeight, depthWidth, 2))

          newDepthData = depthData[:,:,0]+depthData[:,:,1]*256   
          # Convert frame data to 1mm units
          newDepthData = (newDepthData * valueScale).astype('uint16')
          newDepthData[newDepthData == 0] = 451
          newDepthData[newDepthData > 445] = 445

          

          normalized_image = (newDepthData / 4).astype('uint8')

          
          normalized_image = cv2.equalizeHist(normalized_image)
          #depthedge=cv2.Canny(normalized_image, 100, 200)
          outputDepthImage = cv2.cvtColor(normalized_image, cv2.COLOR_GRAY2BGR)

          roi_top_left = (125, 45)  # Top-left coordinate of the ROI rectangle
          roi_bottom_right = (550, 435)  # Bottom-right coordinate of the ROI rectangle
          
##############################################
          gray = cv2.cvtColor(newColorData, cv2.COLOR_BGR2GRAY)
          _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
          roi = binary[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
          roi_blurred = cv2.GaussianBlur(roi, (5, 5), 0)
          contours, _ = cv2.findContours(roi_blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

          for contour in contours:
            for point in contour:
                point[0][0] += roi_top_left[0]
                point[0][1] += roi_top_left[1]

          for contour in contours:
            if cv2.contourArea(contour) > 100: 
                
                cv2.drawContours(newColorData, [contour], -1, (0, 255, 0), 1)
                
                M = cv2.moments(contour)
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                cv2.putText(newColorData, str(newDepthData[centroid_y, centroid_x]), (centroid_x, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                
                rect = cv2.minAreaRect(contour)

                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(newColorData, [box], 0, (255, 255, 0), 1)

                _, (width, height), _ = rect
                object_length = max(width, height)
                object_length_text = f"{object_length:.2f} units"

                length_mm = float(object_length*0.8620689655)
                mm_text = f"{length_mm:.2f} mm"


                
                x = centroid_x - int(object_length / 2)
                y = centroid_y + 20
                cv2.putText(newColorData, object_length_text, (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(newColorData, mm_text, (x, y+15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

          cv2.drawContours(newColorData, contours, -1, (0, 255, 0), 1)
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
        
          
          
          
          cv2.namedWindow("xyzviewer", cv2.WINDOW_NORMAL)
          cv2.setMouseCallback("xyzviewer", mouse_callback)  
          #cv2.imshow("xyzviewer", newData)
          #cv2.imshow("xyzviewer", outputDepthImage)
          #cv2.imshow("xyzviewer", edge)
          cv2.imshow("xyzviewer", newColorData)
          #cv2.imshow("xyzviewer", newDepthData)
          #cv2.imshow("xyzviewer", gray)
          
          
          if key == ESC or key == q:
            cv2.destroyAllWindows()
            break
pipe.stop()
