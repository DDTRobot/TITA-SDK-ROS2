<p align="center"><strong>tita_bringup</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>


​	机器人的节点启动器，通过启动此节点中的不同的 launch 来改变机器人的各个模式。

## Basic Information

| Installation method | Supported platform[s]      |
| ------------------- | -------------------------- |
| Source              | Jetpack 6.0 , ros-humble |

------

## Build Package

```bash
# if have extra dependencies
colcon build
ros2 launch tita_bringup base.launch.py
```
