#!/bin/bash
# Check if the commandline argument "full_install" is passed
if [$1 = "full_install"]
then
    sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon
    sudo apt install python3-wstool
fi
cd .. # You are in the workspace src folder
rm -f .rosinstall
rm -f .rosinstall.bak
wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/alexandros.rosinstall
wstool update -t .
rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}
cd .. # You are in the workspace root
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
