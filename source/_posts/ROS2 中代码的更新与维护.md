---
title: ROS2中代码的更新与维护
categories: ROS2
tags:
  - ROS2
abbrlink: 128206af
date: 2022-05-02 15:30:16
---



安装vcs

```bash
sudo apt-get install python3-vcstool
```

下载代码

```Bash
#在ros2_ws目录下运行，代码会存在src目录下（以ros2仓库作为示例）
mkdir -p ros2_ws/src
cd ros2_ws
vcs import src < ros2.repos
```

更新已经下载的代码

```Bash
#在ros2_ws目录下运行
vcs pull src
```

导出当前src目录下的git仓库信息

```Bash
vcs export src >> tmp.repos
```

<!--more-->