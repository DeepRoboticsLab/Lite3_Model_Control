#!/bin/bash

# 获取版本参数
ros_version=$1

# 检查版本参数是否为空
if [ -z "$ros_version" ]; then
  echo "未提供 ROS 版本参数"
  exit 1
fi

# 安装依赖项
sudo apt update
sudo apt install -y libyaml-cpp-dev
sudo apt install -y libeigen3-dev
sudo apt install -y liblcm-dev
sudo apt install -y libglm-dev
sudo apt install -y ros-$ros_version-rosbash
sudo apt install -y ros-$ros_version-xacro
sudo apt install -y ros-$ros_version-rviz
sudo apt install -y ros-$ros_version-controller-interface
sudo apt install -y ros-$ros_version-gazebo-ros-control 
sudo apt install -y ros-$ros_version-joint-state-controller 
sudo apt install -y ros-$ros_version-effort-controllers 
sudo apt install -y ros-$ros_version-joint-trajectory-controller

echo "安装完成"
