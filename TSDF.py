import numpy as np

def raycasting(voxelx,camPose,camIns):
    pass

def Voxel2WorldPosition(index):
    """
    :param index: voxel index (i,j,k)
    :return: world system position ndarray [x,y,z,1]^T
    """
    i,j,k = index
    x = i*1+0.5
    y = j * 1 + 0.5
    z = k * 1 + 0.5
    return np.array([x,y,z,1]).reshape((4,1))
    pass

def sdf(volume,pose,intrinsic,points3d):
    """
    :param volume: pre_defined grid
    :param pose: from world to frame
    :param points3d: frame points cloud
    :return: a weighted volume
    """
    x,y,z = volume.shape
    for i in range(x):
        for j in range(y):
            for k in range(z):
                voxel = volume[i][j][k]
                position = Voxel2WorldPosition((i,j,k))
                camera_pos = np.dot(pose, position)
                uvd = np.dot(intrinsic, camera_pos)


    pass