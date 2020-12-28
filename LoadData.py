import numpy as np
import cv2
import os

"""
5000 = 100 cm
"""

def show(img):
    cv2.namedWindow('show', cv2.WINDOW_NORMAL)
    cv2.imshow('show', img)
    cv2.waitKey(0)
    cv2.destroyWindow('show')

def LoadRGB(floderpath):
    files = os.listdir(floderpath)
    files.sort()
    RGB = []
    for filename in files:
        img_path = floderpath+'/'+filename
        img = cv2.imread(img_path)
        RGB.append(img)
    # check
    return RGB

def LoadDepth(floderpath):
    files = os.listdir(floderpath)
    files.sort()
    Depth = []
    for filename in files:
        img_path = floderpath+'/'+filename
        img = cv2.imread(img_path, -1)
        depth = img.astype(np.float32)

        Depth.append(depth)
    # check
    return Depth

if __name__ == '__main__':
    # LoadRGB('/home/waveviewer/PycharmProjects/KinectFusion/TestData/RGB')
    LoadDepth('/home/waveviewer/PycharmProjects/KinectFusion/TestData/Depth')
