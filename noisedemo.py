import csv

dt = []

i = 0

with open('hhhhh.txt', 'r') as fin:
    i += 1
    print(i)
    while True:
        try:
            t = fin.readline().strip()
            if t:
                dt.append(float(t))
            else:
                break
        except EOFError:
            break

import numpy as np

array = np.array(dt, np.float16)
print(array.shape)
np.save('noisem',array)