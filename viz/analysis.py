import os
import numpy as np
from matplotlib import pyplot as plt

with open('./fly.txt', 'r') as f:
    lines = f.readlines()

vz_set, vz_out, z_feb = [], [], []
for line in lines[0:-1]:
    line = line.strip('\n')
    line_list = line.split(' ')
    vz_set.append(float(line_list[3]))
    vz_out.append(float(line_list[7]))
    z_feb.append(float(line_list[5]))

time = len(vz_set)
step = np.arange(time)
#print vz_out
plt.subplot(211)
plt.plot(step, vz_set, step, vz_out)
plt.subplot(212)
plt.plot(step, z_feb)
plt.show()


