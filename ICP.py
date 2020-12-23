import numpy as np
from sklearn.neighbors import KDTree
import cv2

def ICP_point_to_plane():
    pass


def ICP(P1, P2, max_iter=20, dist_thresh=20):
    '''
    :param P1:  n*3
    :param P2:  n*3
    :param max_iter: 100
    :return R,t
    '''

    len = P2.shape[0]
    try:
        kdt = KDTree(P1, metric='euclidean')
    except:
        print('length of P1', len(P1))
        print('2D点的数据过少')
        raise Exception
    Trans = np.hstack([np.eye(2), np.zeros((2, 1))])
    print(Trans)
    pre_num = 0
    pre_Trans = Trans
    pre_mean_error = 100
    thresh = 80
    for i in range(max_iter):
        if (i > 2):
            thresh = dist_thresh
        P2_tmp = Trans.dot(np.hstack((P2, np.ones((len, 1)))).T).T
        dist, ind_src = kdt.query(P2_tmp, k=1, return_distance=True)
        dist = np.squeeze(dist)
        ind_src = np.squeeze(ind_src)

        ind_dst = np.arange(0, len)

        ind = np.squeeze(dist < thresh)
        mean_error = np.mean(dist[ind])
        if (mean_error > pre_mean_error):
            break

        pre_mean_error = mean_error
        ind_src = ind_src[ind]
        ind_dst = ind_dst[ind]
        cur_num = np.array(ind_src).shape[0]
        pre_num = max(pre_num, cur_num)

        pre_Trans = Trans
        Trans = cv2.estimateAffine2D(P2[ind_dst], P1[ind_src])[0]

    # print('ICP error',mean_error)
    # print('iteraton times',i)
    P2_tmp = pre_Trans.dot(np.hstack([P2, np.ones((len, 1))]).T).T
    dist, ind_src = kdt.query(P2_tmp, k=1, return_distance=True)
    dist = np.squeeze(dist)
    ind_src = np.squeeze(ind_src)
    ind_dst = np.arange(0, len)
    ind = np.squeeze(dist < thresh)
    ind_src = ind_src[ind]
    ind_dst = ind_dst[ind]
    return ind_src, ind_dst

if __name__ == '__main__':
    ICP(np.zeros((3,3)),np.zeros((3,3)),2)