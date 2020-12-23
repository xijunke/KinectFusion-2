import numpy as np
import cv2
import time

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

def NormalMap(Points,h,w):
    start_time = time.time()
    normal_map = []
    for i in range(h):
        normal_map.append([])
        for j in range(w):
            index = i*w+j
            vec1 = Points[:,index]
            if i+1 >= h:
                vec2 = np.array([0.0,0.0,0.0])
            else:
                index2 = (i+1)*w+j
                vec2 = Points[:, index2]
            if j+1 >= w:
                vec3 = np.array([0.0,0.0,0.0])
            else:
                index3 = i*w+j+1
                vec3 = Points[:,index3]
            normal = np.cross(vec2-vec1,vec3-vec1)
            normal_map[i].append(normal)
    print('Time Used in NormalMap', time.time()-start_time)
    return normal_map


if __name__ == '__main__':
    Points = np.zeros((3, 800))
    norm = NormalMap(Points, 20, 40)


