import numpy as np

if __name__ == '__main__':
    a = np.arange(5)
    b = np.vstack((a,np.zeros((1,5))))
    ind = [1,2,4]
    c = b[:,ind]
    print(c)