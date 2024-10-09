<p align="center"><strong>tita_interfaces</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

​	用于存放机器人中的 ros 自定义消息接口。 

## Basic Information

| Installation method | Supported platform[s]    |
| ------------------- | ------------------------ |
| Source              | Jetpack 6.0 , ros-humble |

------

## tita_locomotion_interfaces

自定义的运动控制指令

- header： ros 包的头消息
- height：机器人两个轮子中心的平均高度到 base_link 的距离
- split：机器人两个轮子分腿的距离
- pose：机器人驱赶的姿态信息
- twist：机器人底盘速度控制信息

## Build Package

```bash
# if have extra dependencies
# apt install <libdepend-dev>
colcon build --packages-select tita_locomotion_interfaces
```
