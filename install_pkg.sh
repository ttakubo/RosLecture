#!/bin/sh

sudo apt-get install ros-${ROS_DISTRO}-joint-state-publisher* -y
sudo apt-get install liburdfdom-tools -y
sudo apt-get install ros-${ROS_DISTRO}-moveit* -y
sudo apt-get install ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-position-controllers ros-${ROS_DISTRO}-joint-trajectory-controller -y
sudo apt-get install ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control -y
sudo apt-get install ros-${ROS_DISTRO}-navigation -y
sudo apt-get install ros-${ROS_DISTRO}-joy -y
