from ObTypes import *
from Property import *
import Pipeline
import StreamProfile
from Error import ObException
import cv2
import numpy as np
import sys

from primesense import openni2#, nite2
from primesense import _openni2 as c_api

np.set_printoptions(threshold=sys.maxsize)

dist = np.array([0.448018097, 3.60273275,  -0.000818341298, 0.00307180999, -1.49815960e+02])
mtx = np.array([[2.34515245e+03, 0, 5.54018738e+02],[0, 2.36459745e+03, 4.52216370e+02],[0, 0, 1]])
q = 113
ESC = 27
alpha   = 0.7

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


pipe.start(config, None)
pipe.getDevice().setBoolProperty(OB_PY_PROP_DEPTH_MIRROR_BOOL, False)


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_value = newDepthData[y, x]
        print("Depth at ({}, {}): {}".format(x, y, depth_value))

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
        print(colorWidth, colorHeight)


        if colorData is not None and depthData is not None:
          newColorData = colorData
          newColorData = cv2.imdecode(newColorData,1)
          if newColorData is not None:
            newColorData = np.resize(newColorData,(colorHeight, colorWidth, 3))
            #newColorData = cv2.undistort(newColorData, mtx, dist, None, mtx)
          depthData = np.resize(depthData,(depthHeight, depthWidth, 2))

          newDepthData = depthData[:,:,0]+depthData[:,:,1]*256
          #print(newDepthData)
          #print(len(newDepthData))        
          # Convert frame data to 1mm units
          newDepthData = (newDepthData * valueScale).astype('uint16')
          normalized_image = (newDepthData / 2).astype('uint8')
          
          outputDepthImage = cv2.cvtColor(normalized_image, cv2.COLOR_GRAY2RGB) 
          
          if colorHeight != depthHeight:
            outputDepthImage = cv2.resize(outputDepthImage,(colorWidth,colorHeight))

          if newColorData is not None:
            newData = newColorData
          if outputDepthImage is not None:
            newData = outputDepthImage
          if newColorData is not None and outputDepthImage is not None:
            newData = cv2.addWeighted(newColorData, (1 - alpha), outputDepthImage, alpha, 0)
          
          cv2.namedWindow("xyzviewer", cv2.WINDOW_NORMAL)

          cv2.imshow("xyzviewer", newData)
          cv2.setMouseCallback("xyzviewer", mouse_callback)  
          
          if key == ESC or key == q:
            cv2.destroyAllWindows()
            break
pipe.stop()
