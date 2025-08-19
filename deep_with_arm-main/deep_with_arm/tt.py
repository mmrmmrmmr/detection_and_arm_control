def loadtxtmethod(filename):
    data = np.loadtxt(filename,dtype=np.float32,delimiter=',')
    return data

import numpy as np

ans_pos = np.array((1.1,1.1,1.1))

d = loadtxtmethod("/home/mmr/Desktop/ddd/ros/src/deep_with_arm/deep_with_arm/color.txt")
print(d)
Lower = d[0]
Upper = d[1]

data = loadtxtmethod("/home/mmr/Desktop/ddd/ros/src/deep_with_arm/deep_with_arm/color.txt")
 

pos_camera_raw = np.array(ans_pos.tolist() + [1])
msg = (np.matmul(data, pos_camera_raw[:, None]))[0:3].reshape(3).tolist()