## About
This project includes simulation of binocular laser profile scanner with rotational table.

## Requirements
* ROS Noetic
* ROS Control and ROS Controllers
* Gazebo
* Rviz
* OpenCV
* PCL Viewer

## Installation
* Follow the [ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu) steps in the link according to **Desktop-Full Install**. After that, **ROS**, **Gazebo**, **RViz** and **OpenCV** installations will be completed.

* Install the [ROS Control](https://wiki.ros.org/ros_control) and [ROS Controllers](https://wiki.ros.org/ros_controllers) packages by executing:
```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

* Install PCL Viewer by executing:
```bash
sudo apt install libpcl-dev pcl-tools
```

## Environment Setup
Source following bash script for using ROS terminal commands. If you do not want to type this command every time you open a terminal, you can add it to `~/.bashrc`.
```bash
source /opt/ros/noetic/setup.bash
```

Creating catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## How to Use?
* Clone repository and build package:
```bash
cd ~/catkin_ws/src/
git clone https://github.com/aliyildiz1/binocular_laser_profile_scanner.git
cd ..
catkin_make
```

* Launch simulation:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch binocular_laser_profile_scanner scanner_simulation.launch 
```

* Open another terminal and start scanning:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun binocular_laser_profile_scanner main.py
```

After the scanning is completed, the point cloud is saved in the `~/catkin_ws/src/binocular_laser_profile_scanner/scans` folder.
To visualize point cloud, execute following commands:
```bash
cd ~/catkin_ws/src/binocular_laser_profile_scanner/scans/
pcl_viewer scanned_point_cloud.pcd
```

## Test Video
[![ROS Binocular Laser Profile Scanner](https://github.com/aliyildiz1/binocular_laser_profile_scanner/assets/119592916/0b8d3846-56ec-48db-b346-abf4166033c0)](https://youtu.be/kXHssHaErtY?si=A3ch_5CR5H5_4xQs)
