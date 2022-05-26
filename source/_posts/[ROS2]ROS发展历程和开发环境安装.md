---
title: “[ROS2]ROS发展历程和开发环境安装” #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: ROS发展历程和开发环境安装
---


# ROS2养成计划（一）发展历程和开发环境安装



## ROS发展历程介绍

![](https://gitee.com/shoufei/blog_images/raw/master/20220220195919.png)



2006 年，无比好奇的一群人走在一起，组建了一个机器人研究实验室：**柳树车库（Willow Garage）**

2010年5月26日，人类历史上第一个机器人毕业典礼在硅谷这条蜿蜒的柳树街68号的小路旁，临时搭建的一座帐篷中举办。

![](/home/kevin/kevin_blogs/TMP_images/20220220200101.png)





四年思考的三个问题： 

第一：为什么人类需要机器人？ 

斯科特·哈森的答案是：机器人可以帮助我们提高生产和工作效率。机器人已经在工厂里证明可以大幅度的提供工业的生产效率，在日常生活中，人类也需要机器人提高工作效率和生活质量。而且需要大量的机器人。 

第二：为什么我们至今还没有好的机器人？

 斯科特·哈森答案是：过去的经验告诉我们，造一个机器人非常困难，资本对机器人方向的关注和投入也很少。投资很少是因为市场很小，市场很小是因为机器人能做的事情很有限。没有市场，就没有投资，没有投资就没有人造机器人，没有人造机器人，就没有机器人。所以我们先从造机器人开始。

第三：如何实现人类拥有机器人的梦想？ 

斯科特·哈森答案是：我们正在做这件事，但是我们自己无法实现这个梦想，我们通过构建一个社群，让工程师，研究人员，工业企业参与进来，联合起来，一起来做 。希望有生之年能实现这个梦想，一个人人拥有机器人的梦想。 



ROS是PR2项目其中的软件部分，车库采用同时与斯坦福大学人工智能实验室吴恩达教授合作的形式，将STAIR项目中的系统软件Switchyard为基础，构建ROS，并采用开源软件方式，邀请世界上（其实一开始主要是美国）感兴趣的人能参与进来。



ROS的含义

与个人电脑的操作系统发挥的作用类似。

**为了更有效的管理计算机硬件，并提高计算机程序的开发效率，就出现了计算机操作系统。**

**为了更有效的管理机器人的硬件，并提高机器人应用程序的开发效率，就出现了机器人操作系统。**



ROS最早的原型是在美国**斯坦福大学**的**人工智能实验室**开发的，后来硅谷这家叫“柳树车库”的机器人公司与斯坦福大学合作，将ROS应用到公司开展的一个个人机器人PR2项目中。



**摩根·奎格利**（吴恩达的博士生）发现将机械臂操控、导航、视觉等各种功能集成在一个机器人上非常不容易，因此那时他就考虑并采用了**“分布式”**的方式，来连接不同的模块。这一概念被成功应用到后来的ROS中。



 2007年，摩根·奎格利和吴恩达将STAIR的成果发表在IEEE 国际机器人与自动化会议上，文章的题目是**《STAIR:Hardware and Software Architecture》**，软件系统的名称是**Switchyard**。这个**Switchyard就是ROS前身**。



2009年摩根·奎格利、吴恩达和柳树车库机器人公司的工程师们，在当年的IEEE国际机器人与自动化会议上发表了**《ROS: An Open-Source Robot Operating System》**，正式向外界介绍ROS。 正如文章中说强调的：  **ROS is not an operating system in the traditional sense of process management and scheduling; rather, it provides a structured communications layer above the host operating systems of a heterogenous compute cluster.** （译文：ROS不是传统意义上的操作系统，不是用于进程管理和调度，而是构建在其它操作系统之上的一种结构化的通讯层。）



 2012年，摩根·奎格利终于博士毕业了，他选择作为首席架构师，与布莱恩·格基（Brian Gerkey，CEO）、罗伯特·弗里德曼（Roberta Friedman， CFO）、凯特·考尼（Nate Koenig， CTO）一起创建了“**开源机器人基金会 (OpenSource Robot Foundation，OSRF)**”。



在“开源机器人基金会”的推动下，ROS先后发布了**Groovy，Hydro，Indigoo，Jade，Kinetic Kame，Logger head，Melodic Morenia**等版本。 除了负责ROS的开发和维护，“开源机器人基金会”同时推动**机器人仿真平台Gazebo**的开发。



随着ROS社群不断的壮大，在**2012年5月**，“开源机器人基金会”组织了第一届ROS开发者大会（简称ROSCon）。



从2016年起陆续有来自中国的团队做简短的分享，比如来自深圳的蓝胖子机器人、来自山东济南的汤尼机器人。



![](https://gitee.com/shoufei/blog_images/raw/master/20220220200209.png)



**2013年到2014年初，是车库机器人研发的最后一个阶段**。外界传闻很多，车库内部很多人心绪不定，对于PR2的未来背地里也是议论纷纷。车库的房东斯科特·哈森，决定停止继续投入资金， PR2的研究也停下来。**车库的各路人马，正式从“机器人研究的阵地”退下来，吹响“机器人商业化”的号角。**经过8年努力，“柳树车库系”终于全力冲进了机器人市场。



从“柳树车库”直接衍生出的公司正在改变着世界。一些前期的公司包括：

hiDOF：机器人与自动化软件咨询公司，2013被Google收购。

IPI（Industrial Perception Inc.）：利用视觉辅助工业机械臂搬运货物，2013被Google收购。

OpenCV：一家非盈利机构，开发开源视觉和机器学习算法。

PCL：一家非盈利机构，开发点云处理算法。

OPF（Open Perception Foundation）：一家非盈利机构，致力于2D/3D 数据的处理。

 



![](https://gitee.com/shoufei/blog_images/raw/master/20220220200237.png)



OSRF（Open Source Robotics Foundation）：一家非盈利机构，致力于机器人开源软件的研究、教育、产品开发。

Open Robotics：由OSRF创立的一家盈利机构，致力于机器人开源软件的对外服务、软件开发、机器人开发咨询服务。

Redwood Robotics：低成本协作机械臂，2013年被Google收购.

Suitable Technologies：远程遥控机器人beam。

Unbounded Robotics：开发低成本移动机器人平台。

Fetch Robotics：由Unbounded Robotics团队重新打造的一款面向仓储物流的移动机器人平台。



硅谷似乎不缺这样的故事。

[【IT文化杂谈】硅谷，仙童与“八叛徒”的故事](https://blog.csdn.net/chengzhf/article/details/108933516)



## ROS的版本迭代

[https://www.ros.org/blog/getting-started/#](https://www.ros.org/blog/getting-started/#)

ROS1版本

![](https://gitee.com/shoufei/blog_images/raw/master/20220220200300.png)



ROS2的版本

![](https://gitee.com/shoufei/blog_images/raw/master/20220220200325.png)

## ROS2相关网站

ROS官网

[https://www.ros.org/](https://www.ros.org/)

ROS 2 Galactic Geochelone介绍网站

[https://docs.ros.org/en/galactic/Releases/Release-Galactic-Geochelone.html](https://docs.ros.org/en/galactic/Releases/Release-Galactic-Geochelone.html)

ROS问答社区

[https://answers.ros.org/questions/](https://answers.ros.org/questions/)

各个ROS版本的文档

[http://docs.ros.org/](http://docs.ros.org/)

ROS Index （这里可以搜索ROS包的相关信息）

[https://index.ros.org/](https://index.ros.org/)

ROS Discourse（ros开发者讨论聚集地）

[https://discourse.ros.org/](https://discourse.ros.org/)

ROS开发者大会（演讲材料及视频）

[https://roscon.ros.org/world/2021/](https://roscon.ros.org/world/2021/)

ROS2设计思路博客

[https://design.ros2.org/](https://design.ros2.org/)

ROS技术讲座

[https://vimeo.com/osrfoundation/videos](https://vimeo.com/osrfoundation/videos )  （需要翻墙）

OpenRoboticsOrg 的twitter （需要翻墙）

[https://twitter.com/openroboticsorg](https://twitter.com/openroboticsorg)

Robot Operating System (ROS) 的twitter （需要翻墙）

[https://twitter.com/rosorg](https://twitter.com/rosorg)



![](https://gitee.com/shoufei/blog_images/raw/master/20220220200425.png)

## 开发环境搭建

- [ROS2开发环境搭建](https://blog.csdn.net/shoufei403/article/details/122981102) 

## 运行小乌龟

[https://docs.ros.org/en/galactic/Tutorials/Turtlesim/Introducing-Turtlesim.html](https://docs.ros.org/en/galactic/Tutorials/Turtlesim/Introducing-Turtlesim.html)

分别在两个窗口运行下面的命令

```Bash
source ~/.bashrc
ros2 run turtlesim turtlesim_node
```



```Dockerfile
ros2 run turtlesim turtle_teleop_key
```

## 仿真SLAM和导航的演示

**源码安装turtlebot3**

[[ROS2\]源码安装turtlebot3用于调试（简易版）](https://blog.csdn.net/shoufei403/article/details/123035407) 

添加TURTLEBOT3_MODEL等环境变量

```Bash
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```



**导航演示**

启动导航软件

```Apache
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True 
```

启动仿真环境

```Apache
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

先在`Rviz`上点击`2D Pose Estimate`按钮，然后在地图上（机器人大概所在位置）点击按住移动鼠标调整方向。这个操作是为了初始化机器人位置。再点击Rviz菜单栏上的`2D Goal Pose`按钮，然后在地图上点击按住移动鼠标设置目标点及其方向。



**SLAM建图演示**

启动仿真环境

```Shell
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

启动建图程序

```Apache
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

启动手动控制程序

```Dockerfile
ros2 run turtlebot3_teleop teleop_keyboard
```

地图保存

```Dockerfile
ros2 run nav2_map_server map_saver_cli -f ~/map
```



**遇到的问题：**

1. 运行ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True 报错

```Bash
error while loading shared libraries: libspqr.so.2.0.2
```

解决办法（不太优雅但可行）:

```Apache
cd /usr/lib/x86_64-linux-gnu/
sudo cp libspqr.so.2.0.9 libspqr.so.2.0.2
sudo rm libspqr.so
sudo rm libspqr.so.2
sudo ln -s libspqr.so.2.0.2 libspqr.so
sudo ln -s libspqr.so.2.0.2 libspqr.so.2
```





**可参考的学习资料：**

鱼香ROS：拥有一键安装ROS1 ROS2的神器

[https://fishros.com/#/fish_home](https://fishros.com/#/fish_home)

官方教程：最官方

[https://docs.ros.org/en/galactic/index.html](https://docs.ros.org/en/galactic/index.html)

新一代机器人操作系统ROS 2技术文档：这个教材不简单，写的比官方教材还细致。感谢大佬。

[https://www.zhihu.com/column/c_1348897856313581568](https://www.zhihu.com/column/c_1348897856313581568)

古月居ros2教程：一个字 “稳”

[https://www.guyuehome.com/Blog/index/category/14/p/2](https://www.guyuehome.com/Blog/index/category/14/p/2)

Automatic Addison : 一个不错的博客

[https://automaticaddison.com/](https://automaticaddison.com/)

ROS史话

[https://www.guyuehome.com/Blog/index/category/13/p/2](https://www.guyuehome.com/Blog/index/category/13/p/2)



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![](https://gitee.com/shoufei/blog_images/raw/master/shoufei_qr.jpg)