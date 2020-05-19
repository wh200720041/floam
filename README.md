# FLOAM 
## Fast LOAM (Lidar Odometry And Mapping)

This work is an optimized version of A-LOAM and LOAM with the computational cost reduced by up to 3 times.
This code is modified from [LOAM](https://github.com/laboshinl/loam_velodyne) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) .

**Modifier:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

## 1. Modification Highlights
This includes some optimization on the original implementation
1. Analytic methods is used instead of auto differentiation. This is performed on se3
2. Use linear motion prediction model to estimate the initial pose
3. Laser odometry and laser mapping are merged 
4. A dynamic local map is used instead of global map, in order to save memory cost. Based on massive experiments, this only has slight influence on the performance. 

## 2. Evaluation
### 2.1. Computational efficiency evaluation
Computational efficiency evaluation (based on KITTI dataset):
Platform: Intel® Core™ i7-8700 CPU @ 3.20GHz 
| Dataset                                      | ALOAM                      | FLOAM                  |
|----------------------------------------------|----------------------------|------------------------|
| `KITTI`                                      | 151ms                      | 59ms                   |

Localization error:
| Dataset                                      | ALOAM                      | FLOAM                  |
|----------------------------------------------|----------------------------|------------------------|
| `KITTI sequence 00`                          | 0.55%                      | 0.51%                  |
| `KITTI sequence 02`                          | 3.93%                      | 1.25%                  |
| `KITTI sequence 05`                          | 1.28%                      | 0.93%                  |

### 2.2. localization result
<img width="65%" src="/img/kitti_example.gif"/>

### 2.3. mapping result
<p align='center'>
<a href="https://youtu.be/w_R0JAymOSs">
<img width="65%" src="/img/floam_mapping.gif"/>
</a>
</p>

## 3. Prerequisites
### 3.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 3.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 3.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

## 4. Build 
### 4.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/floam.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
### 4.2 Download test rosbag
Download [KITTI sequence 05](https://drive.google.com/open?id=18ilF7GZDg2tmT6sD5pd1RjqO0XJLn9Mv) or [KITTI sequence 07](https://drive.google.com/open?id=1VpoKm7f4es4ISQ-psp4CV3iylcA4eu0-)

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by 
```
sudo apt-get install unzip 
```

And then copy the file 2011_09_30_0018.bag into ~/catkin_ws/src/floam/dataset/ (this may take a few minutes to unzip the file)
```
	cd ~/catkin_ws/src/floam/dataset/
	unzip ~/Downloads/2011_09_30_0018.zip
```

### 4.3 Launch ROS
```
    roslaunch floam floam.launch
```
if you would like to create the map at the same time, you can run (more cpu cost)
```
    roslaunch floam floam_mapping.launch
```


## 5. Test other sequence
To generate rosbag file of kitti dataset, you may use the tools provided by 
[kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) or [kitti2bag](https://github.com/tomas789/kitti2bag) 

## 6.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).

