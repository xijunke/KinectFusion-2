import numpy as np
import cv2
from FramePreprocessing import DepthMap,PointCloud,NormalMap
from ICP import ICP_point_to_point
# Load Frames
RGB = []
Depth = []
frame_num = len(RGB)

# Camera Intrinsic 3*3
Intrinsic = np.array([[918.774170584315,	0,	0],
                      [1.93012762035975,	921.829519864905,	0],
                      [960.063190260167,	540.614346667332,	1]]).T
inverse = np.linalg.inv(Intrinsic)

AllFramesPointsCloud = []
# preprocessing
# Set first frame as world system
first_Depthmap = DepthMap(Depth[0])
first_Points3D = PointCloud(first_Depthmap, inverse)
AllFramesPointsCloud.append(first_Points3D)

# other frames produce 3D points
for i in range(1, frame_num):
    depthmap = DepthMap(Depth[i])
    points3D = PointCloud(depthmap, inverse)
    AllFramesPointsCloud.append(points3D)

# ICP find Trans between neighboring frames
AllFramesPose = [] # from world to frame i
pose = ICP_point_to_point(AllFramesPointsCloud[0], AllFramesPointsCloud[1])
AllFramesPose.append(pose)
for i in range(1, frame_num-1):
    pose = ICP_point_to_point(AllFramesPointsCloud[i], AllFramesPointsCloud[i+1])
    temp = np.dot(AllFramesPose[i-1], pose)
    AllFramesPose.append(temp)

# Global Transform matrix
# from frame i to world
Global_Trans = []
for i in range(frame_num-1):
    pose = np.linalg.inv(AllFramesPose[i])
    Global_Trans.append(pose)

# init volume
Volume = np.zeros((150, 150, 150))

# Transform all points cloud to world system
GlobalPointsCloud = []
GlobalPointsCloud.append(first_Points3D)
for i in range(1, frame_num):
    points = np.vstack((AllFramesPointsCloud[i], np.ones((1, AllFramesPointsCloud[i].shape[1]))))
    pose = Global_Trans[i-1]
    pointsset = np.dot(pose, points)
    GlobalPointsCloud.append(pointsset[:3])


# update volume via TSDF



# visualization via MarchingCubes