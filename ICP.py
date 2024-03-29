import numpy as np
from sklearn.neighbors import KDTree
import cv2

def ICP_point_to_plane():
    pass

def FindRigidTransform(points_set_P, points_set_Q):
    """
    :param points_set_P: 3D points cloud, ndarray 3*N
    :param points_set_Q: 3D points cloud, ndarray 3*N
    :return: Trans [R|t] from P to Q, shape=(3,4)
    """
    P = np.mean(points_set_P, axis=1, keepdims=True)
    Q = np.mean(points_set_Q, axis=1, keepdims=True)
    # mass center transform
    X = points_set_P - P
    Y = points_set_Q - Q
    M = X.dot(Y.T)

    U, Sigma, Vt = np.linalg.svd(M)
    R = np.dot(Vt.T, U.T)
    t = Q - R.dot(P)
    trans = np.hstack((R, t))
    print('Trans', trans)
    return trans
    pass

def FindMatchingPairs(points_set_P,points_set_Q,pose, thresh=20):
    """
    :param points_set_P: 3D points cloud, ndarray 3*N
    :param points_set_Q: 3D points cloud, ndarray 3*N
    :param pose: Trans [R|t] from P to Q shape=(3,4)
    :param thresh: distance threshold of filtering matching pairs
    :return: matching pairs index -> ind_P,ind_Q
    """
    P = np.vstack((points_set_P, np.ones((1, points_set_P.shape[1]))))
    print()
    P_projection = pose.dot(P)
    kdt = KDTree(points_set_Q, metric='euclidean')
    dist, ind = kdt.query(P_projection, k=1, return_distance=True)
    print('dist')
    print(dist)
    ind_P = []
    ind_Q = []
    mean_error = 0
    for i in range(len(dist)):
        if dist[i] < thresh:
            ind_P.append(i)
            ind_Q.append(ind[i])
            mean_error += dist[i]
    # mean_error /= len(ind_P)
    return ind_P, ind_Q
    pass

def ICP_point_to_point(points_set_P, points_set_Q):
    """
    :param points_set_P: 3D points cloud, ndarray 3*N
    :param points_set_Q: 3D points cloud, ndarray 3*N
    :return: Trans [R|t] from P to Q, shape is (4,4),the last row is [0|1]
    iteration times = 20, find the best trans matrix between neighboring frames
    """
    iter_times = 20
    pose = FindRigidTransform(points_set_P, points_set_Q)
    ind_P,ind_Q = FindMatchingPairs(points_set_P, points_set_Q, pose)
    matching_num = len(ind_P)
    print('First matching num',matching_num)
    for i in range(iter_times):
        temp_P = points_set_P[:,ind_P]
        temp_Q = points_set_Q[:,ind_Q]
        temp_pose = FindRigidTransform(temp_P, temp_Q)
        temp_ind_P, temp_ind_Q = FindMatchingPairs(points_set_P, points_set_Q, pose)
        temp_matching_num = len(temp_ind_P)
        if temp_matching_num > matching_num:
            pose = temp_pose
            ind_P = temp_ind_P
            ind_Q = temp_ind_Q
            matching_num = temp_matching_num
        else:
            break
    print('pose',pose)
    return np.vstack((pose, np.arrray([0,0,0,1])))
    pass

if __name__ == '__main__':
    pass