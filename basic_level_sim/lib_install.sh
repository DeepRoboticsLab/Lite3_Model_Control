#!/bin/bash

# 获取版本参数
ros_version=$1

# 检查版本参数是否为空
if [ -z "$ros_version" ]; then
  echo "未提供版本参数"
  exit 1
fi

# 安装 ROS 的依赖
sudo apt-get update
sudo apt-get install -y ros-$ros_version-rosbash
sudo apt-get install -y ros-$ros_version-xacro
sudo apt-get install -y ros-$ros_version-rviz
sudo apt-get install -y ros-$ros_version-gazebo* ros-$ros_version-controller-interface ros-$ros_version-gazebo-ros-control
sudo apt-get install -y ros-$ros_version-robot-state-publisher ros-$ros_version-joint-state-controller 
sudo apt-get install -y ros-$ros_version-position-controllers ros-$ros_version-effort-controllers ros-$ros_version-jointjectory-controller
sudo apt-get install -y ros-$ros_version-joint-state-publisher-gui

# 更新环境变量
source /opt/ros/$ros_version/setupash

echo "安装完成"
