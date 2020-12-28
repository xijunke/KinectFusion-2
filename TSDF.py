import numpy as np

def raycasting(voxelx,camPose,camIns):
    pass

def Voxel2WorldPosition(index):
    """
    :param index: voxel index (i,j,k)
    :return: world system position ndarray [x,y,z,1]^T
    """
    i, j, k = index
    x = i * 1 + 0.5
    y = j * 1 + 0.5
    z = k * 1 + 0.5
    return np.array([x,y,z,1]).reshape((4,1))
    pass

def tsdf(volume,weight,pose,intrinsic,points3d):
    """
    :param volume: pre_defined grid
    :param pose: from world to frame
    :param points3d: frame points cloud
    :return: a weighted volume
    """
    max_truncation = 2
    x,y,z = volume.shape
    for i in range(x):
        for j in range(y):
            for k in range(z):
                position = Voxel2WorldPosition((i,j,k))
                camera_pos = np.dot(pose, position)
                uvd = np.dot(intrinsic, camera_pos)
                if uvd[0][0] >= 0 and uvd[0][0] < 1920 and uvd[1][0] >= 0 and uvd[1][0] < 1080 :
                    index = uvd[1][0]*1920+uvd[0][0]
                    depth = points3d[2][index]
                    sdf = depth - position[2][0]
                    if sdf > 0:
                        tsdf = min((1, sdf/max_truncation))
                    else:
                        tsdf = min((-1,sdf/max_truncation))
                    old_weight = weight[i][j][k]
                    old_tsdf = volume[i][j][k]
                    # weight update without normalmap
                    new_weight = old_weight + 1
                    new_tsdf = (old_tsdf*old_weight+tsdf*new_weight)/new_weight
                    weight[i][j][k] = new_weight
                    volume[i][j][k] = new_tsdf
    pass