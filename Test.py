import numpy as np

if __name__ == '__main__':
    a = []
    for i in range(3):
        a.append([])
        for j in range(3):
            a[i].append(j)
    print(a[1][1])