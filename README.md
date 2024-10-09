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
