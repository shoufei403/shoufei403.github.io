---
title: 下载TurtleBot4源码来学习
categories: ROS2
tags:
  - ROS2
  - TurtleBot4
abbrlink: ae5fa866
date: 2022-06-12 20:03:16
---


![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20220512131200.png)



新一代的`turtlebot`终于来了。[撒花撒花]


`turtlebot`是开源的低成本移动机器人平台，希望以低成本的方式帮助更多的开发者学习实践机器人技术。

这次发布的`TurtleBot 4`共有两个版本：`TurtleBot 4 Standard`和`TurtleBot 4 Lite`。


它们是以`iRobot Create3`作为移动底盘，配备了`OAK-D`摄像头和`2D`激光雷达。计算平台是`Raspberry Pi 4`，并运行`ROS2`。



`TurtleBot 4 `软件运行在`Ubuntu 20.04 LTS (Focal Fossa) `上，目前只支持`ROS2 Galactic`版本。它是目前第一个支持`ROS2`的`Turtlebot`。



开发`turtlebot4`的公司叫`clearpathrobotics`，下面是它在`github`上的主页：

[https://github.com/clearpathrobotics](https://github.com/clearpathrobotics)

<!--more-->

项目介绍网址：

[https://clearpathrobotics.com/turtlebot-4/](https://clearpathrobotics.com/turtlebot-4/)

[https://clearpathrobotics.com/blog/2022/05/clearpath-robotics-launches-turtlebot-4/](https://clearpathrobotics.com/blog/2022/05/clearpath-robotics-launches-turtlebot-4/)



`turtlebot`的软件仓库：

[https://github.com/turtlebot](https://github.com/turtlebot)



`turtlebot4`涉及到的软件和硬件资源：

[https://turtlebot.github.io/turtlebot4-user-manual/overview/resources.html](https://turtlebot.github.io/turtlebot4-user-manual/overview/resources.html)



`Create® 3`的官方介绍文档：

[https://iroboteducation.github.io/create3_docs/](https://iroboteducation.github.io/create3_docs/)



`turtlebot4`用户手册：

[https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html)



## TurtleBot4主要仓库

`turtlebot4`仓库中包含了一些基础功能包。这些基础功能包会被`turtlebot4_robot` 和`turtlebot4_simulator`使用。`turtlebot4_robot`运行在物理机上，`turtlebot4_simulator`则是仿真环境的代码。

以防大家想手动安装各个仓库。下面列出了`turtlebot4`相关的主要仓库。

如果想源码编译，则需要安装必要的依赖：

**depthai**

安装其依赖：

```bash
sudo apt-get install libdwarf-dev
sudo apt install clang-format-10
```

[https://github.com/luxonis/depthai-core/releases/tag/v2.15.4](https://github.com/luxonis/depthai-core/releases/tag/v2.15.4)

下载好后可以参考其`README.md`文件来编译安装。

编译命令：

```bash
cd depthai-core-v2.15.4
cmake -S. -Bbuild -D'BUILD_SHARED_LIBS=ON' -D'CMAKE_INSTALL_PREFIX=/usr/local'
sudo cmake --build build --target install
```



编译报错及其解决措施

报错信息：

```bash
CMake Error at shared/depthai-shared.cmake:39 (if):
  if given arguments:

    "STREQUAL" "-"

  Unknown arguments specified
```



```bash
CMake Error at shared/depthai-bootloader-shared.cmake:33 (if):
  if given arguments:

    "STREQUAL" "-"

  Unknown arguments specified
```

解决措施：

在相应文件报错处更改如下

```cmake
        #string(SUBSTRING ${statusCommit} 0 1 status)
        #if((${status}) STREQUAL "-")
        #    message(FATAL_ERROR "Submodule 'depthai-bootloader-shared' not initialized/updated. Run 'git submodule update --init --recursive' first")
        #endif()
```

将执行报错的位置注释掉。



**与`turtlebot4`相关的仓库如下：**

`turtlebot4`仓库包含4个子文件夹

 `turtlebot4_description` 功能包中包含`turtlebot4`的`URDF` 描述文件和相应的`mesh`文件。

`turtlebot4_msgs`中自定义了一些消息。

`turtlebot4_navigation` 功能包里包含启动`SLAM`和`Navigation`的相关代码。同时它也包含一个基于`Python`的`TurtleBot 4 Navigator`节点。

`turtlebot4_node`功能包包含人机交互和蓝牙遥控的相关代码。

```bash
git clone https://github.com/turtlebot/turtlebot4.git -b galactic
```



`turtlebot4_robot`仓库

`turtlebot4_base`需要运行在实机上，主要包含与`Raspberry Pi`外设（如GPIO）交互的相关代码。

`turtlebot4_bringup`包含启动机器人软件的`launch`文件和配置文件。

`turtlebot4_diagnostics`主要用于诊断话题发布频率和其他机器人状态数据（如电池电压，轮组状态等等），然后将诊断数据发布出来。在`rqt_robot_monitor`中可查看相关数据。

`turtlebot4_tests`中包含一些`TurtleBot 4 `的系统测试脚本。

```bash
git clone https://github.com/turtlebot/turtlebot4_robot.git -b galactic
```



`turtlebot4_desktop`仓库

`turtlebot4_viz`中包含在`Rviz2`中查看机器人状态的`launch`文件和配置文件。

```bash
git clone https://github.com/turtlebot/turtlebot4_desktop.git -b galactic
```



`turtlebot4_simulator`仓库

该仓库中包含了仿真`turtlebot4`的相关文件。这里使用的仿真软件是`Ignition Edifice`。这是下一代的`gazebo`。

```bash
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b galactic
```



`turtlebot4_examples`仓库

```bash
git clone https://github.com/turtlebot/turtlebot4_examples -b galactic
```



`turtlebot4_tutorials`仓库

```bash
git clone https://github.com/turtlebot/turtlebot4_tutorials -b galactic
```



`irobot_create_msgs`仓库（自定义消息的仓库）

```bash
git clone https://github.com/iRobotEducation/irobot_create_msgs -b galactic
```



`create3_sim`仓库（包含irobot create3仿真模型相关文件）

```bash
git clone https://github.com/iRobotEducation/create3_sim.git -b main
```



与luxonis相关的依赖

```bash
git clone https://github.com/luxonis/depthai-ros.git -b main
```

```bash
git clone https://github.com/luxonis/depthai-ros-examples.git -b main
```

下面的仓库是工具软件。

`turtlebot4-hardware`仓库（硬件说明）

```bash
git clone https://github.com/turtlebot/turtlebot4-hardware
```



`turtlebot4_setup`中包含设置`turtlebot4`的脚本

```bash
git clone https://github.com/turtlebot/turtlebot4_setup
```



`robot_upstart`功能包主要是用于设置launch文件的系统自启动。方便开机时就启动相应的程序。

```bash
git clone https://github.com/clearpathrobotics/robot_upstart.git -b foxy-devel
```

设定自启动示例

```bash
ros2 run robot_upstart install turtlebot4_bringup/launch/lite.launch.py --job turtlebot4 --rmw rmw_cyclonedds_cpp --rmw_config /etc/cyclonedds_rpi.xml
```

解除自启动示例

```bash
ros2 run robot_upstart uninstall turtlebot4
```

查看服务是否已启动

```bash
systemctl | grep turtlebot4
```

如果服务启动了，将打印

```bash
turtlebot4.service loaded active running "bringup turtlebot4"
```

停止服务

```bash
sudo systemctl stop turtlebot4.service
```



**注意：**

- 下载慢的话就使用`https://ghproxy.com/`代码工具：

使用示例

```
git clone https://ghproxy.com/https://github.com/turtlebot/turtlebot4.git
```

- 如果编译会卡死，建议一次只编译一部分功能包。在文件夹里新建空的`COLCON_IGNORE`文件就可以屏蔽编译。



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
