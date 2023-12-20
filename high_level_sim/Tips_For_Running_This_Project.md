# Tips for running this project

## error 1: gazebo_gui-3  process has died [pid 2255, exit code 139, cmd /opt/ros/melodic/lib/gazebo_ros/gzclient __name:=gazebo_gui __log
按照如下[链接](https://blog.csdn.net/gls_nuaa/article/details/122142356) 仍然解决不了的话，卸载后装Gazebo11，按照此[教程](https://zhuanlan.zhihu.com/p/526057704) 卸载Gazebo9 安装Gazebo11。

## error 2: Could not find a package configuration file provided by "Python3" with any of the following names: Python3Config.cmake  python3-config.cmake
升级cmake 版本>=3.12 [升级教程](https://blog.csdn.net/qq_27350133/article/details/121994229)。
## error 3 usr/bin/ld: cannot find -llcm collect2: error: ld returned 1 exit status
```
sudo apt install liblcm-dev
```
## error 4 Resource not found: xacro
```
sudo apt-get install ros-melodic-xacro
```

## error 5 sh: 1: rosrun: not found
```
sudo apt install ros-melodic-rosbash
```
## tip   6  有些时候需要升级/安装python
请使用如下脚本
```
sudo apt-get install python3.8 python3.8-dev python3.8-distutils python3.8-venv
```




