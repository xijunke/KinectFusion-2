import numpy as np
import cv2
from FramePreprocessing import DepthMap,PointCloud,NormalMap
from ICP import ICP_point_to_point
# Load Frames
RGB = []
Depth = []
frame_num = len(RGB)

# Camera Intrinsic 3*3
Intrinsic = np.eye(3)
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
AllFramesPose = []
for i in range(frame_num-1):
    pose = ICP_point_to_point(AllFramesPointsCloud[i], AllFramesPointsCloud[i+1])
    AllFramesPose.append(pose)

# Global Transform matrix
Global_Trans = []
pose = np.eye(4)
for i in range(frame_num-1):
    pose = np.dot(pose, np.linalg.inv(AllFramesPose[i]))
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