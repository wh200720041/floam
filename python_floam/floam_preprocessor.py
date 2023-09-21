import os
 
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