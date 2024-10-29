<p align="center"><strong>TITA-SDK-ROS2</strong></p>
<p align="center"><a href="https://github.com/DDTRobot/TITA-SDK-ROS2/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

​	TITA Ubuntu 系统的 SDK Demo.

## Basic Information

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------

## Published

|       ROS Topic        |                   Interface                    | Frame ID |    Description    |
| :--------------------: | :--------------------------------------------: | :------: | :---------------: |
| `command/user/command` | `tita_locomotion_interfaces/msg/LocomotionCmd` |  `cmd`   | 用户 SDK 控制指令 |

## Build Package

```bash
mkdir -p tita_sdk/src
cd tita_sdk/src
git clone https://github.com/DDTRobot/TITA-SDK-ROS2.git
colcon build
source install/setup.bash
ros2 launch tita_bringup sdk_launch.py
```

## Config 

|       Param       |      Range      | Default |                    Description                     |
| :---------------: | :-------------: | :-----: | :------------------------------------------------: |
|  `sdk_max_speed`  |      `3.0`      |  `3.0`  |              机器的速度上限，3.0 m/s                  |
| `turn_max_speed`  |      `6.0`      |  `6.0`  |              旋转速度上限，6.0 rad/s                  |
|  `pub_freq`       |  [100.0,170.0]|  `170`  | 发布频率，单位 Hz，范围 [100.0,170.0]              |    

## Quick Start

* Use the remote control to put the robot in standing mode .

* Press the small buttons on the right side of the small screen on the remote control, press it in the middle, appear, the "mode select" interface
![/tita_select_mode](./docs/img/1280X1280.PNG)
* In the use button to push down, select the USE-SDK Mode Press, the robot will be automatically executed, and the use-SDK will take over the control permissions.
![/tita_select_mode](./docs/img/1280X1280%20(1).PNG)

## FAQ
1. If ros2 Launch tita_bringup sdk_launch.py ​​exits, the robot will still be executed automatically, unless the user-SDK control permissions are released
2. If the robot has no response ,angular.z value in sdk_command_node.cpp is too small