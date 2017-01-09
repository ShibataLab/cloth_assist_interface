# cloth_assist_interface

This repository contains a ROS package to work with Baxter research robot, Kinect V2 and Motion Analysis Hawk motion capture system. This package is being used to collect various datasets for publications related to clothing assistance.

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

* First install Ubuntu 14.04 on the Desktop PC and setup ROS indigo following steps 1 and 2 in this [page](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).
* Create a new ROS wokspace and install the necessary Baxter SDK software following steps 3 to 7 in this [page](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).
* Install libfreenect and the necessary graphics card drivers by following the instructions in this [page](https://github.com/code-iai/iai_kinect2).
* Download the iai_kinect2 ROS package in the same workspace as Baxter SDK and install it by following the instructions in this [page](https://github.com/code-iai/iai_kinect2).
* Install ar_track_alvar by opening a terminal and typing the following command:
  ```
  sudo apt-get install ros-indigo-ar-track-alvar ros-indigo-ar-track-alvar-msgs
  ```
* Install zeromq library by opening a terminal and typing the following command:
  ```
  sudo apt-get install libzmq3 libzmq3-dev libzmqpp3 libzmqpp3-dev
  ```
* Download the cloth_assist_interface ROS package in the same ROS workspace as above and compile it:
  ```
  cd ~/ros_ws/src
  git clone https://github.com/ShibataLab/Interface.git
  cd ..
  catkin_make
  ```
* If catkin make exits without any errors, then you can start using the programs by running:
  ```
  source ~/ros_ws/devel/setup.bash
  ```
  ```
  ./baxter.sh
  ```

## Usage

### Utility programs

* CPP files to interface with Kinect V2
* CPP files to interface with Motion Capture system
* Python files to work with Baxter robot
* CPP and Python codes for simultaneous data recording
* Launch files for AR marker tracking
* RQt and rviz config files for GUI and Data visualization

## Troubleshooting
