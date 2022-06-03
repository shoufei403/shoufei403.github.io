---
title: ROS2中代码的更新与维护 #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2] #文章标签，可空，多标签请用格式，注意:后面有个空格
# description: ROS2 中代码的更新与维护方法
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