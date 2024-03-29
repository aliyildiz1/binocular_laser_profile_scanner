## About
This project includes simulation of binocular laser profile scanner with rotational table.

## Requirements
* ROS2 Humble
* ROS2 Control and ROS2 Controllers
* Gazebo
* RViz2
* OpenCV
* PCL Viewer

## Installation
Follow the [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) steps in the link according to **Desktop Install**. After that, **ROS**, **RViz** and **OpenCV** installations will be completed.

Install the [ROS2 Control](https://github.com/ros-controls/ros2_control) and [ROS2 Controllers](https://github.com/ros-controls/ros2_controllers) packages by executing:
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

Install Gazebo and Gazebo ROS2 packages by executing:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
```

Install joint state publisher gui by executing:
```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

Install xacro by executing:
```bash
sudo apt install ros-humble-xacro
```

Install PCL Viewer by executing:
```bash
sudo apt install libpcl-dev pcl-tools
```

## Environment Setup
Source following bash script for using ROS terminal commands. If you do not want to type this command every time you open a terminal, you can add it to `~/.bashrc`.
```bash
source /opt/ros/humble/setup.bash
```

Creating ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/
colcon build
```

## How to Use?
Clone repository and build package:
```bash
cd ~/ros2_ws/src/
git clone --single-branch -b ros2-humble https://github.com/aliyildiz1/binocular_laser_profile_scanner.git
cd ..
colcon build
```

In order to load texture to projector, you need to copy texture image to `/usr/share/gazebo-11/media/materials/textures/`. Copy texture image by executing:
```bash
sudo cp ~/ros2_ws/src/binocular_laser_profile_scanner/projector_textures/line_laser_5p.png /usr/share/gazebo-11/media/materials/textures
```

Launch simulation:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch binocular_laser_profile_scanner simulation_launch 
```

Open another terminal and start scanning:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run binocular_laser_profile_scanner main
```

After the scanning is completed, the point cloud is saved in the `~/ros2_ws/src/binocular_laser_profile_scanner/scans` folder.
To visualize point cloud, execute following commands:
```bash
cd ~/ros2_ws/src/binocular_laser_profile_scanner/scans/
pcl_viewer scanned_point_cloud.pcd
```

## Test Video
[![ROS Binocular Laser Profile Scanner](https://github.com/aliyildiz1/binocular_laser_profile_scanner/assets/119592916/0b8d3846-56ec-48db-b346-abf4166033c0)](https://youtu.be/kXHssHaErtY?si=A3ch_5CR5H5_4xQs)
