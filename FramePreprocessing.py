import numpy as np
import cv2

def DepthMap(deepth):
    dmap = cv2.bilateralFilter(deepth,5,35,35)
    return dmap

def PointCloud(depth,camIn):
    h, w = depth.shape
    num = h*w
    DepthMap = depth.reshape((1, num))
    Inverse = np.linalg.inv(camIn)
    u = np.zeros((1, w))
    v = np.arange(w)
    boardv = np.tile(v,(1,h))

    for i in range(1, h):
        u = np.concatenate((u,np.full((1,w),i)))
    uvdMap = np.vstack(u,boardv,DepthMap)
    print('chech uvdMap shape', uvdMap.shape)

    Points = np.dot(Inverse,uvdMap)
    print('chech PointsCloud shape', Points.shape)
    pass
