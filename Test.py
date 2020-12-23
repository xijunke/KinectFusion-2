import numpy as np

if __name__ == '__main__':
    v = np.arange(5)
    print(v)
    b = np.tile(v,(1,2))
    a = np.full((1,10),2)
    c = np.full((1,10),-3)
    print(b)
    print(a)
    d = np.vstack((a,b,c))
    print(d)