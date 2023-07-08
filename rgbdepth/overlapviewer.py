from ObTypes import *
from Property import *
import Pipeline
import StreamProfile
from Error import ObException
import cv2
import numpy as np
import sys

q = 113
ESC = 27
alpha   = 0.7

pipe = Pipeline.Pipeline(None, None)

config = Pipeline.Config()
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
          if newColorData is not None:
            newColorData = np.resize(newColorData,(colorHeight, colorWidth, 3))
        
          depthData = np.resize(depthData,(depthHeight, depthWidth, 2))
          
          # Convert frame data from 8bit to 16bit
          newDepthData = depthData[:,:,0]+depthData[:,:,1]*256          
          # Convert frame data to 1mm units
          newDepthData = (newDepthData * valueScale).astype('uint16')
          normalized_image = (newDepthData / 2).astype('uint8')
          
          # Convert depth frame data GRAY to RGB
          outputDepthImage = cv2.cvtColor(normalized_image, cv2.COLOR_GRAY2RGB) 
          
          # Need to scale to the same resolution when the depth is not the same as the resolution of the color
          if colorHeight != depthHeight:
            outputDepthImage = cv2.resize(outputDepthImage,(colorWidth,colorHeight))
            
            
          if newColorData is not None:
            newData = newColorData
          if outputDepthImage is not None:
            newData = outputDepthImage
          if newColorData is not None and outputDepthImage is not None:
            newData = cv2.addWeighted(newColorData, (1 - alpha), outputDepthImage, alpha, 0)
          
          cv2.namedWindow("SyncAlignViewer", cv2.WINDOW_NORMAL)

          cv2.imshow("SyncAlignViewer", newData)

          if key == ESC or key == q:
            cv2.destroyAllWindows()
            break
pipe.stop()
