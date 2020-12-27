import numpy as np
import cv2
import time

def DepthMap(deepth):
    # 双边滤波
    dmap = cv2.bilateralFilter(deepth,5,35,35)

    # 如果深度值越界，需要提前处理，不要传递给后续点云生成

    pass
    return dmap

def PointCloud(depth,Inverse):
    """
    将每帧照片生成点云，对应点的RGB信息分开存储
    :param depth: DepthMap of one frame
    :param camIn: Camera Intrinsic inverse K^{-1}
    :return: 3D Points Set shape->3*N
    """
    h, w = depth.shape
    num = h*w
    DepthMap = depth.reshape((1, num))
    # Inverse = np.linalg.inv(camIn)
    u = np.zeros((1, w))
    v = np.arange(w)
    boardv = np.tile(v, (1, h))

    for i in range(1, h):
        u = np.concatenate((u, np.full((1, w), i)))
    uvdMap = np.vstack(u, boardv, DepthMap)
    print('check uvdMap shape', uvdMap.shape)

    Points = np.dot(Inverse, uvdMap)
    print('check PointsCloud shape', Points.shape)
    pass

def NormalMap(Points,h,w):
    # 越界的法向量被设置成了(0,0,0),可能需要处理
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
                vec3 = Points[:, index3]
            normal = np.cross(vec2-vec1, vec3-vec1)
            normal_map[i].append(normal)
    print('Time Used in NormalMap', time.time()-start_time)
    return normal_map


if __name__ == '__main__':
    Points = np.zeros((3, 800))
    norm = NormalMap(Points, 20, 40)


