---
title: 数据回放伴侣Rviz+plotjuggler
categories: ROS2
tags:
  - ROS2
  - Rviz
  - plotjuggler
abbrlink: 7a7b7752
date: 2022-05-26 15:30:16
---



## Plotjuggler简介

`PlotJuggler`是一个类似于`rqt_plot`的基于Qt的数据可视化工具。但`PlotJuggler`拥有更强大和好用的功能。你可以导入文本文件让它显示文本文件中的数据。你也可以导入ros的bag包，它能自动解析bag包中的数据。并可以回放bag包的数据，然后用`Rviz`来显示数据。`PlotJuggler`的功能有很多，这里只介绍几种我常用的功能。应该足以应付日常的机器人开发调试工作。



`Plotjuggler`官方网址：

[https://www.plotjuggler.io/](https://www.plotjuggler.io/)

`github`地址：  

[https://github.com/facontidavide/PlotJuggler](https://github.com/facontidavide/PlotJuggler)

<!--more-->

## 安装Plotjuggler

注意一下，如果是配合ROS1来使用，推荐安装`PlotJuggler 2.X.X`，如果是配合ROS2来使用，则推荐安装`PlotJuggler 3.X.X`。  

**基于二进制AppImage文件安装**

可以在二进制文件下载网页直接下载。  

下载网页：[https://github.com/facontidavide/PlotJuggler/releases](https://github.com/facontidavide/PlotJuggler/releases)

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/756d230f6f852178e0f5f874cfbf2cd3.png)

文件下载好后需要给文件增加执行权限，然后双击即可打开使用。



**ROS二进制文件安装（推荐）**

使用下面的命令：

```bash
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
```



运行`Plotjuggler`

对于ROS1

```bash
rosrun plotjuggler plotjuggler
```

对于ROS2

```bash
ros2 run plotjuggler plotjuggler
```

升级新版本时也可以使用该命令

```bash
sudo apt-get update
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
```





**源码安装**

用这种方式的话，你可能是想对`Plotjuggler`的代码进行修改。源码编译分两种情况，**与ROS无关的源码编译**和**需要与ROS联合使用的源码编译**。  

> 如果拉取github代码很慢，可尝试使用github下载加速工具[https://ghproxy.com/](https://ghproxy.com/) 。



- 与ROS无关的源码编译  

克隆仓库  

```bash
git clone https://github.com/facontidavide/PlotJuggler.git
```

安装编译前的依赖  

```bash
sudo apt -y install qtbase5-dev libqt5svg5-dev libqt5websockets5-dev libqt5opengl5-dev libqt5x11extras5-dev libprotoc-dev
```

执行编译  

```bash
 mkdir build; cd build
 cmake ..
 make
 sudo make install
```

**注意：使用该方式编译是没有ROS相关的插件的。**



- 需要与ROS联合使用的源码编译

对于ROS1

1. 建立工作空间并拉取代码

```bash
 mkdir -p ~/ws_plotjuggler/src
 cd ~/ws_plotjuggler/src
 git clone https://ghproxy.com/https://github.com/PlotJuggler/plotjuggler_msgs.git
 git clone https://ghproxy.com/https://github.com/facontidavide/PlotJuggler.git
 git clone https://ghproxy.com/https://github.com/PlotJuggler/plotjuggler-ros-plugins.git
```

2. 解决依赖并编译  

```bash
cd ~/ws_plotjuggler
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make
```

3. 启动`Plotjuggler`

```bash
source devel/setup.bash
roslaunch plotjuggler_ros plotjuggler.launch
```



对于ROS2  Galactic

1. 建立工作空间并拉取代码

```bash
 mkdir -p ~/ws_plotjuggler/src
 cd ~/ws_plotjuggler/src
 git clone https://ghproxy.com/https://github.com/PlotJuggler/plotjuggler_msgs.git -b ros2
 git clone https://ghproxy.com/https://github.com/facontidavide/PlotJuggler.git
 git clone https://ghproxy.com/https://github.com/PlotJuggler/plotjuggler-ros-plugins.git -b galactic
```

2. 解决依赖并编译  

```bash
cd ~/ws_plotjuggler
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
colcon build
```

3. 启动`Plotjuggler`

```bash
source install/setup.bash
ros2 run plotjuggler plotjuggler
```



## 用Plotjuggler显示机器人路径

**保存机器人轨迹为CSV文件**

使用下面的示例代码来存储机器人的位置。存储其他数据也可以用类似的方法。

```c++
#include <fstream>
#include <ios>
#include <iostream>
#include <string>
#include <vector>

struct Pose
{
    double x;
    double y;
}

void saveCvsFile(std::string file_name, std::vector<Pose> path)
{
    std::ofstream outfile;
    outfile.open(file_name.c_str(), std::ios::trunc);
    outfile << "x"
        << ","
        << "y"
        << ","
        << "yaw"
        << ","
        << "num" << std::endl;
    int num_count = 0;
    for (int i = 0; i < path->size(); ++i)
    {
        outfile << path->at(i).x << "," << path->at(i).y << ","  
            << path->at(i).yaw << "," << num_count << std::endl;
        num_count++;
    }
    outfile.close();
}

```



**Plotjuggler显示CSV文件路径**

打开`Plotjuggler`，选中`Data`加载`CSV`文件。  

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/b2fe15ab9bdb082126c2245363dcf1a2.png)



选择`num`作为x轴  

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/fd327a9d04c05745e542b9a255c1b535.gif)



## 显示ROS bag中的数据

**录制bag的命令**

```bash
ros2 bag record -o bagname 话题名称
```

示例：

```bash
ros2 bag record -o turtlebot /turtle1/cmd_vel /turtle1/pose
```



**Plotguggler加载ROS2 bag**

这里以加载ROS2记录的bag为例。对于ROS1 bag，操作过程也是类似的。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/40ff8286cbd961a57c6532a19a24cf06.png)



ros bag中的数据是按时间顺序记录的，所以在`Plotguggler`中可以随意拖动进度条查看不同时间点的数据内容。这个功能对调试来说太方便了。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/8ebf74a46b3b2d6e3fb6c8d153690771.gif)





## 订阅ROS话题

点击下面的是`start`按钮开始订阅话题数据，点击`stop`结束话题订阅。然后左下方就能看到已经记录好的数据了。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/8d515d4c876aa8a1dc523242f5e714b2.png)



**Plotguggler回放ROS2 bag，Rviz显示数据**

目前该功能在ROS2环境中还是有问题。`Plotguggler`的github中已经开了相关的`Issue`，但作者似乎修改的不彻底。仍然有崩溃的问题。`Plotguggler 2.x.x`的版本在ROS1环境下亲测可以正常使用。拖动进度条，可按任意节奏播放记录的话题数据。

`Issue`网址:

[[[ROS2/foxy\]Segmentation Fault when trying to topic Re-Publisher](https://github.com/facontidavide/PlotJuggler/issues/477#)](https://github.com/facontidavide/PlotJuggler/issues/477)





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。



