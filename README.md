# Interface

This repository contains a ROS package to work with Baxter research robot, Kinect V2 and Motion Analysis Hawk motion capture system. This package is being used to collect various datasets for publications made by ShibataLab.

## Dependencies
This package requires several dependencies to run successfully:

##### Hardware
1. Desktop PC with USB 3.0 Port and NVidia/Intel graphics card.
2. Baxter research robot
3. Kinect for Windows V2
4. Hawk motion capture system

##### Software
1. Ubuntu 14.04 operating system
2. Baxter SDK
3. libfreenect and Kinect V2 ROS bridge
4. ar_track_alvar ros package
5. ZeroMQ library

## Install
The installation instructions are as followed:

1. First install Ubuntu 14.04 on the Desktop PC and setup ROS indigo following steps 1 and 2 in this [page](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).
2. Create a new ROS wokspace and install the necessary Baxter SDK software following steps 3 to 7 in this [page](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).
3. Install libfreenect and the necessary graphics card drivers by following the instructions in this [page](https://github.com/code-iai/iai_kinect2).
4. Download the iai_kinect2 ROS package in the same workspace as Baxter SDK and install it by following the instructions in this [page](https://github.com/code-iai/iai_kinect2).
5. Install ar_track_alvar by opening a terminal and typing the following command:
  ```
  sudo apt-get install ros-indigo-ar-track-alvar ros-indigo-ar-track-alvar-msgs
  ```
6. Download the Interface ROS package in the same ROS workspace as above and compile it:
  ```
  cd ~/ros_ws/src
  git clone https://github.com/ShibataLab/Interface.git
  cd ..
  catkin_make
  ```
7. If catkin make exits without any errors, then you can start using the programs by running:
  ```
  source ~/ros_ws/devel/setup.bash
  ```
  ```
  ./baxter.sh
  ```

## Usage


## Troubleshooting
