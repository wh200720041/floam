#!/usr/bin/env python3
import os
import numpy as np
import cv2

color = cv2.imread('1.png')
depth = cv2.imread('1.pgm', -1)

 
# Function to Get the current
# working directory
def current_path():
    print("Current working directory before")
    print(os.getcwd())
    print()
 
 
current_path()
 
# Changing the CWD
os.chdir('/workspaces/vscode_ros2_workspace/build/floam')
 
# Printing CWD after
current_path()
files = os.listdir(os.curdir)
# print(files)
import sys
sys.path.append(os.getcwd())

###################################################

import floam_preprocessing

FloamPreprocessor_=floam_preprocessing.LaserProcessingClass()

scan_line = 64;
vertical_angle = 2.0;
scan_period= 0.1;
max_dis = 60.0;
min_dis = 2.0;


FloamPreprocessor_.init_python(scan_line,vertical_angle, scan_period,max_dis,min_dis)

cx = 325.5
cy = 253.5
fx = 518.0
fy = 519.0
depthScale = 1000.0


#    depth = depth[:, :, 0]
#https://stackoverflow.com/questions/36421437/reading-pgm-images-with-cv2-in-python
    

'''
根据slam14讲 公式 5.5
（X,Y,Z)是三维点的坐标
u = fx*X/Z + cx
v = fy*Y/Z + cy  
要先知道每个点的深度值Z，然后根据u v fx fy cx cy
算出对应的X Y
图片尺寸为 H x W
u 的范围为[0, W-1]
v 的范围为[0, H-1]

Z是深度

X = (u-cx)*Z/fx
Y = (v-cy)*Z/fy

这样一个深度图可以生成XYZ图，还有RGB信息
点云矩阵是个 n x 6 的矩阵，对应 X,Y,Z,B,G,R
n = H x W = 480 x 640 = 307200
'''

H, W = color.shape[:2]

u, v = np.meshgrid(np.arange(0, W), np.arange(0, H))

Z = depth/depthScale
X = (u-cx)*Z/fx
Y = (v-cy)*Z/fy

X = X.reshape(307200, 1)
Y = Y.reshape(307200, 1)
Z = Z.reshape(307200, 1)

#RGB = 255*np.ones((307200, 3)).astype(np.float64)

b = color[:, :, 0]
g = color[:, :, 1]
r = color[:, :, 2]

b = b.reshape(307200, 1)
g = g.reshape(307200, 1)
r = r.reshape(307200, 1)

#RGB = np.hstack((b, g, r))
INTENSITY =  X.reshape(307200, 1)
pcd = np.hstack((X,Y,Z, INTENSITY))

FloamPreprocessor_.featureExtraction_python(pcd)

print(FloamPreprocessor_.get_edge())