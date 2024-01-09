# M-detector

## 1.Introduction

**M-detector** is a moving event detection package, which determines if a point from LiDAR is moving immediately after its arrival, resulting in a point-by-point detection with a latency of just several microseconds. M-detector is designed based on occlusion principles and can be used in different environments with various types of LiDAR sensors.

### **1.1 Related paper**

Our related papers has been accepted by Nature Communications. [Moving Event Detection from LiDAR Stream Points](https://www.nature.com/articles/s41467-023-44554-8).

If our code is used in your project, please cite our paper.

### **1.2 Related video**

Our accompanying videos are now available on **YouTube** (click below images to open) and [**Bilibili**](https://www.bilibili.com/video/BV1ke411i7t7/?share_source=copy_web).

<div align="center">
<a href="https://www.youtube.com/watch?v=SYaig2eHV5I" target="_blank"><img src="img/cover.bmp" alt="video" width="60%" /></a>
</div>

### 1.3 Developers

The codes of this repo are contributed by:
[Huajie Wu (吴花洁)](https://github.com/HuajieWu99), [Yihang Li (李一航)](https://github.com/yihangHKU) and [Wei Xu (徐威)](https://github.com/XW-HKU)

## 2. Prerequisites

### 2.1 **Ubuntu** and **ROS**

Ubuntu ≥ 18.04.

ROS     ≥ Melodic. Follow [[ROS Installation](http://wiki.ros.org/ROS/Installation)]

### 2.2 **PCL** and **Eigen**

PCL      ≥ 1.8

`sudo apt install libpcl-dev`

Eigen    ≥ 3.3.4

`sudo apt install libeigen3-dev`

### 2.3 **livox_ros_driver**

Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

*Remarks:*

* Since the M-detector support Livox serials LiDAR firstly, so the **livox_ros_driver** must be installed and **sourced** before run any M-detector launch file.
* How to source? The easiest way is add the line `source $Livox_ros_driver_dir$/devel/setup.bash` to the end of file `~/.bashrc`, where `$Livox_ros_driver_dir$` is the directory of the livox ros driver workspace (should be the `ws_livox` directory if you completely followed the livox official document).

### 2.4 TBB

Install gcc-9 g++-9

`sudo add-apt-repository ppa:ubuntu-toolchain-r/test`

`sudo apt update`

`sudo apt install gcc-9 g++-9`

`cd /usr/bin`

`sudo rm gcc g++`

`sudo ln -s gcc-9  gcc`

`sudo ln -s g++-9 g++`

Follow [[TBB Installation](https://solarianprogrammer.com/2019/05/09/cpp-17-stl-parallel-algorithms-gcc-intel-tbb-linux-macos/)] (**Note:** change the gcc-9.1/g++-9.1 to gcc-9/g++-9)

## 3. Build

Clone the repository and catkin_make:

`cd ~/catkin_ws/src`

`git clone git@github.com:hku-mars/M-detector.git`

`catkin_make`

`source devel/setup.bash`
(**Note:** change the path for TBB in CMakeList.txt)

## 4. Key Information

### 4.1 Key parameters

```
dataset: 3    #0 for kitti, 1 for nuscenes, 2 for waymo
buffer_delay: 0.1
buffer_size: 100000
points_num_perframe: 30000
depth_map_dur: 0.2
max_depth_map_num: 5
max_pixel_points: 5
frame_dur: 0.1
hor_resolution_max: 0.005
ver_resolution_max: 0.01
```

The parameters are provided in folder "config" for different LiDARs.

For methods of parameters tuning, please follow the section 8 introduced in [[Supplementary Information](https://www.nature.com/articles/s41467-023-44554-8)].

To save the label files, please pass the parameter via the corresponding launch files.

### 4.2 Folder structure for dataset

```
├── XXX (dataset name)
│   ├── bags
│   │   ├── XXX_0000.bag
│   │   ├── ...
│   ├── sequences
│   │   ├── 0000
│   │   │   ├── labels
│   │   │   ├── predictionsx_origin (results in point-out mode with xth parameter file)
│   │   │   ├── predictionsx (in frame-out mode with xth parameter file)
│   │   │   ├── ...
│   │   ├── ...
├── ...
```

The dataset can be downloaded at [[this link](https://drive.google.com/drive/folders/1ASNfrjZB7n9Q-nB4Pm2IwvArFWnTcFAj?usp=drive_link)].

## 5. Directly Run

### 5.1 Run with odometry and point clouds (in local frame)

At first, please run a odometery node, such as [[Fast Lio](https://github.com/hku-mars/FAST_LIO)] (Download Fast Lio provided in Releases into the same location as M-detector's and complie them).

Then:

`roslaunch fast_lio mapping_XXX(for dataset).launch`

`roslaunch m_detector detector_(dataset).launch`

`rosbag play YOURBAG.bag`

### 5.2 Generate the label files for every point

`roslaunch m_detector detector_XXX.launch out_path:="your path for frame-out results" out_origin_path:="your path for point-out results"`

Note: Follow the folder structure introduced before, the `out_path` should be in the format of "(the path to dataset folder)/(dataset name)/sequences/(sequence number)/predictionsx(x is the parameter file's number)/", and the `out_origin_path` should be in the format of "(the path to dataset folder)/(dataset name)/sequences/(sequence number)/predictionsx_origin(x is the parameter file's number)/".

### 5.3 Calculate the IoU of results

`roslaunch m_detector cal_recall.launch dataset:=(0 for kitti, 1 for nuscenes, 2 for waymo, 3 for avia) dataset_folder:="the path to the dataset_folder" start_se:=(the first sequence number for calculation) end_se:=(the last sequence number for calculation) start_param:=(the first parameter file's number for calculation) end_param:=(the last parameter file's number for calculation) is_origin:=(true for point-out results, false for frame-out results)`

Note: Follow the folder structure introduced before, the `dataset_folder` should be the path to dataset folder. This step will calculate all the IoU for all designated results listed in the dataset folder and generate a new folder named "recall" or "recall_origin" containing the results.

## 6. Run with Embedded in FAST LIO

Download the embedded version provided in Releases into a new workspace and complie them.

`roslaunch fast_lio mapping_(dataset).launch`

`rosbag play YOURBAG.bag`

## 7. Rosbag Download

The bags used in paper can be download at [[this link](https://drive.google.com/drive/folders/1ASNfrjZB7n9Q-nB4Pm2IwvArFWnTcFAj?usp=sharing)].

## 8. License

The source code of this package is released under [**GPLv2**](http://www.gnu.org/licenses/) license. We only allow it free for **academic usage**. For commercial use, please contact Dr. Fu Zhang [fuzhang@hku.hk](mailto:fuzhang@hku.hk).

For any technical issues, please contact me via email [wu2020@connect.hku.hk](mailto:wu2020@connect.hku.hk).
