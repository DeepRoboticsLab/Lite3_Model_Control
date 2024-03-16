Lite3_MotionSDK 与 Gazebo 联合仿真
=====

## 环境和依赖库
请提前安装[ROS1](https://www.ros.org/blog/getting-started/#)和[Gazebo11](https://gazebosim.org/docs)。

对于**ROS melodic**，请安装如下功能包：
```bash
sudo apt-get install ros-melodic-rosbash
sudo apt-get install ros-melodic-xacro
sudo apt-get install ros-melodic-rviz
sudo apt-get install ros-melodic-gazebo* ros-melodic-controller-interface ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-robot-state-publisher ros-melodic-joint-state-controller 
sudo apt-get install ros-melodic-position-controllers ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
sudo apt-get install ros-melodic-joint-state-publisher-gui
```

对于**ROS noetic**，请安装如下功能包：
```bash
sudo apt-get install ros-noetic-rosbash
sudo apt-get install ros-noetic-xacro
sudo apt-get install ros-noetic-rviz
sudo apt-get install ros-noetic-gazebo* ros-noetic-controller-interface ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-robot-state-publisher ros-noetic-joint-state-controller 
sudo apt-get install ros-noetic-position-controllers ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
sudo apt-get install ros-noetic-joint-state-publisher-gui
```

## 下载和解压
下载代码，并将*basic_level_sim*文件夹移动到自己的工作空间${your_workspace}下。

## 代码编译
打开一个新的终端，输入：
```bash
cd ${your_workspace}/basic_level_sim
catkin_make
source devel/setup.bash
```

## 在RViz中查看模型
打开一个新的终端：
```bash
cd ${your_workspace}/basic_level_sim
source devel/setup.bash
roslaunch lite3_motion_sim lite3_rviz_display.launch
```

## 仿真步骤
### 1.加载模型 ###
打开一个新的终端：
```bash
cd ${your_workspace}/basic_level_sim
source devel/setup.bash
roslaunch lite3_motion_sim lite3_gazebo_simulation_control.launch
```

### 2.控制启动 ###
打开一个新的终端：
```bash
cd ${your_workspace}/basic_level_sim
source devel/setup.bash
rosrun lite3_motion_sim lite3_motion_sim
```
