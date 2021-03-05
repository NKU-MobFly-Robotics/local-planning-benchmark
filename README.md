# MRPB: Mobile Robot Local Planning Benchmark

**MRPB** is developed aiming to evaluate mobile robot local planning approaches in a unified and comprehensive way. 
It contains a rich set of elaborately designed simulation scenarios, for instance, complex maze environments, 
partially unknown office-like environments, dynamic pedestrians, and so on. 
More features and challenging scenarios will come in the future :wink:.

The following video shows navigation in static, partially unknown, and dynamic scenarios 
<a href="https://youtu.be/X-N0Sf0-ODY" target="_blank"><div align=center><img src="https://upload-images.jianshu.io/upload_images/9115568-09461a7b58f21087.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240" 
alt="MRPB 1.0: A Unified Benchmark for the Evaluation of Mobile Robot Local Planning Approaches" width="512" height="288" border="10" /></div></a>

To run this project in minutes, check [Quick Start](#1-Quick-Start). Please refer to README.md in each folder to learn more about the contents.

Please kindly star :star: this project if it helps you. We take great efforts to develop and maintain it :grin::grin:.

## Table of Contents

* [Quick Start](#1-Quick-Start)
* [Paper](#2-Paper)
* [Setup](#3-Setup)

## 1. Quick Start

The project is developed and tested in Ubuntu 16.04 with ROS Kinetic. Run the following commands to setup:

```
$ sudo apt-get install ros-kinetic-navigation ros-kinetic-teb-local-planner
$ cd ~/catkin_ws/src
$ git clone https://github.com/NKU-MobFly-Robotics/local-planning-benchmark.git
$ cd ../
$ catkin_make
```
and start a simulation (run in a new terminals): 
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch move_base_benchmark move_base_benchmark.launch
```
You will find the costmap and the Pioneer 3-DX mobile robot in ```Rviz```. You can select goals for the robot to reach using the ```2D Nav Goal``` tool.

## 2. Paper

Please cite the following paper if you use this project in your research: 

- [__MRPB 1.0: A Unified Benchmark for the Evaluation of Mobile Robot Local Planning Approaches__](https://arxiv.org/abs/2011.00491), Jian Wen, Xuebo Zhang,
Qingchen Bi, Zhangchao Pan, Yanghe Feng, Jing Yuan, and Yongchun Fang, 2020, _arXiv:2011.00491_.

## 3. Setup

We use [**Actor Collisions Plugin**](https://github.com/osrf/gazebo/tree/gazebo11/examples/plugins/actor_collisions) to give dynamic pedestrians collision properties, so that they can be swept by the laser rangefinder. From the actor_collisions directory
```
$ mkdir build
$ cd build
$ cmake ../
$ make
```
After that, a library named "libActorCollisionsPlugin.so" will be generated in the build directory. Please update the reference path of "libActorCollisionsPlugin.so" in the xxx_dynamic.world files in the gazebo_world/world directory before you use the dynamic world models. For example, open office02_dynamic.world and use "ctrl+F" to find "libActorCollisionsPlugin.so". Then, replace the value of "filename" with the absolute path of "libActorCollisionsPlugin.so" in your build directory of actor_collisions. Each animated actor needs to call this plugin. Therefore, please check all the reference paths of this plugin in the dynamic world models.

Furthermore, we have tested that dynamic pedestrians based on this plugin can be swept by the laser rangefinder only in Gazebo9 and above. The default version of Gazebo in ROS Kinetic is Gazebo7. Therefore, if you use ROS Kinetic, please uninstall Gazebo7 and install Gazebo9
```
1. Uninstall Gazebo7
  $ sudo apt-get remove --purge ros-kinetic-gazebo* 
  $ sudo apt-get remove --purge libgazebo* 
  $ sudo apt-get remove --purge gazebo* 
  $ sudo apt-get remove --purge gazebo-*

2. Add the source and keys 
  $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

3. Install Gazebo9
  $ sudo apt-get update
  $ sudo apt-get install gazebo9
  $ sudo apt-get install libgazebo9-dev
  $ sudo apt-get install ros-kinetic-gazebo9-*

4. Check the version of Gazebo
  $ gazebo -v
```

## Acknowledgement
  We use [**Actor Collisions Plugin**](https://github.com/osrf/gazebo/tree/gazebo11/examples/plugins/actor_collisions) to give dynamic pedestrians collision properties and [**Pioneer-3dx Simulator**](https://github.com/BruceChanJianLe/p3dx) to simulate Pioneer 3-DX mobile robot in Gazebo.

## License
The source code is released under [CC0-1.0](https://choosealicense.com/licenses/cc0-1.0/) license.