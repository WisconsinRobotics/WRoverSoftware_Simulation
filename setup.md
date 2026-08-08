# Introduction
Our simulation stack has two parts which work together:
1) Ubuntu 22.04 running somewhere for ROS2
2) The Unity Game Engine for simulating
physics and visuals

If you are having some graphical issues regarding WSL, please refer to the graphics section for found fixes.

# ROS2 setup
## 1. Install Ubuntu 22.04
It doesn't really matter how Ubuntu 22.04 is installed, it just needs to be installed in some sort of manner on your computer.

For Windows users [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) is recommened.

## 2. Install ROS2 Humble
Please refer to the [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). **Make sure to pick the desktop install and the ROS2 Dev Tools**.

## 3. Configuring Environment
Please install the following packages
```bash
# Install rosbridge_suite
# Allows for TCP communication between ROS2 and ROS# Library
sudo apt-get install ros-humble-rosbridge-server

# General Development Packages
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep2
sudo apt install ros-humble-joint-state-publisher-gui
```



# More Resources
- [ROS2 Humble Wiki](https://docs.ros.org/en/humble/About-ROS.html)
- [ROS# Wiki](https://github.com/siemens/ros-sharp/wiki)
