# FLOAM
## Fast LOAM (Lidar Odometry And Mapping)

This work is an optimized version of A-LOAM and LOAM with the computational cost reduced by up to 3 times.
This code is modified from [LOAM](https://github.com/laboshinl/loam_velodyne) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) .

**Modifier:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

**ROS2 Migration:** [Yi-Chen Zhang](chris7462.github.io/), Isuzu Technical Center of America, USA

## 1. Demo Highlights
Watch our demo at [Video Link](https://youtu.be/PzZly1SQtng)
<p align='center'>
<a href="https://youtu.be/PzZly1SQtng">
<img width="65%" src="/img/floam_kitti.gif"/>
</a>
</p>

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
<p align='center'>
<img width="65%" src="/img/kitti_example.gif"/>
</p>

### 2.3. mapping result
<p align='center'>
<a href="https://youtu.be/w_R0JAymOSs">
<img width="65%" src="/img/floam_mapping.gif"/>
</a>
</p>

## 3. Prerequisites
### 3.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.

ROS2 Foxy. [ROS Installation](https://docs.ros.org/en/foxy/Installation.html)

### 3.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).
Please checkout to 2.0.0 tag. ROS2 migrated version doest not support Ceres 2.1.0 yet.

### 3.3. **PCL**
For PCL library, please install by the following:
```bash
sudo apt-get install libpcl-dev ros-foxy-pcl-ros
```

## 4. Build
### 4.1 Clone repository:
```bash
cd ~/colcon_ws/src
git clone https://github.com/chris7462/floam.git
cd ..
colcon build
source ./install/setup.bash
```
### 4.2 Download test rosbag
Download [KITTI sequence 05](https://drive.google.com/file/d/1eyO0Io3lX2z-yYsfGHawMKZa5Z0uYJ0W/view?usp=sharing) or [KITTI sequence 07](https://drive.google.com/file/d/1_qUfwUw88rEKitUpt1kjswv7Cv4GPs0b/view?usp=sharing)

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by
```
sudo apt-get install unzip
```

And this may take a few minutes to unzip the file
```bash
cd ~/Downloads
unzip ~/Downloads/2011_09_30_0018.zip
```
Then convert the ROS1 bag to ROS2 bag. See [here](https://ternaris.gitlab.io/rosbags/topics/convert.html) for reference

### 4.3 Launch ROS
```bash
ros2 launch floam floam.launch.py
```
if you would like to create the map at the same time, you can run (more cpu cost)
```bash
ros2 launch floam floam_mapping.launch.py
```
If the mapping process is slow, you may wish to change the rosbag speed by replacing "-r 0.5" with "-r 0.2" in your launch file, or you can change the map publish frequency manually (default is 10 Hz)

## 5. Test on other sequence
To generate rosbag file of kitti dataset, you may use the tools provided by
[kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) or [kitti2bag](https://github.com/tomas789/kitti2bag)

## 6. Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).

## 7. Citation
If you use this work for your research, you may want to cite
```
@inproceedings{wang2021,
  author={H. {Wang} and C. {Wang} and C. {Chen} and L. {Xie}},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={F-LOAM : Fast LiDAR Odometry and Mapping},
  year={2020},
  volume={},
  number={}
}
```
