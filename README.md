# MRPB: Mobile Robot Local Planning Benchmark

**MRPB** is developed aiming to evaluate mobile robot local planning approaches in a unified and comprehensive way. 
It contains a rich set of elaborately designed simulation scenarios, for instance, complex maze environments, 
partiall unknown office-like environments, dynamic pedestrians, and so on. 
More features and challenging scenarios will come in the future :wink:.

To run this project in minutes, check [Quick Start](#1-Quick-Start).

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.

## Table of Contents

* [Quick Start](#1-Quick-Start)
* [Paper](#2-Paper)

## 1. Quick Start

The project is developed and tested in Ubuntu 16.04, ROS Kinetic. Run the following commands to setup:

```
sudo apt-get install ros-kinetic-navigation ros-kinetic-teb-local-planner
cd ~/catkin_ws/src
git clone https://github.com/NKU-MobFly-Robotics/local-planning-benchmark.git
cd ../
catkin_make
source devel/setup.bash
```
and start a simulation (run in a new terminals): 
```
roslaunch navigation_simulation robot_diff_drive_in_gazebo.launch
```
You will find the costmap and the Pioneer3-DX robot in ```Rviz```. You can select goals for the robot to reach using the ```2D Nav Goal``` tool.

## 2. Paper

Please cite the following paper if you use this project in your research: 

- [__MRPB 1.0: A Unified Benchmark for the Evaluation of Mobile Robot Local Planning Approaches__](https://arxiv.org/abs/2011.00491), Jian Wen, Xuebo Zhang,
Qingchen Bi, Zhangchao Pan, Yanghe Feng, Jing Yuan, and Yongchun Fang, 2020, _arXiv:2011.00491_.
