import numpy as np
import cv2
from FramePreprocessing import DepthMap,PointCloud,NormalMap
# Load Frames
RGB = []
Depth = []
frame_num = len(RGB)

# Camera Intrinsic 3*3
Intrinsic = np.eye(3)
inverse = np.linalg.inv(Intrinsic)

AllFramesPointsCloud = []
# preprocessing
# First frame as world system
first_Depthmap = DepthMap(Depth[0])
first_Points3D = PointCloud(first_Depthmap, inverse)
AllFramesPointsCloud.append(first_Points3D)

# from points cloud to volume
volume = dict()
for i in range(first_Points3D.shape[1]):
    key = (int(first_Points3D[0][i]),int(first_Points3D[1][i]),int(first_Points3D[2][i]))
    volume[key] = 0

# other frames produce 3D points
for i in range(1, frame_num):
    depthmap = DepthMap(Depth[i])
    points3D = PointCloud(depthmap, inverse)
    AllFramesPointsCloud.append(points3D)

# ICP find Trans between neighboring frames

