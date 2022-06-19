---
title: ROS2基础之Navigation2导航系统
categories: ROS2
tags:
  - ROS2
  - Navigation2
abbrlink: 39d4e548
date: 2022-06-19 22:38:17
---


## Navigation2整体架构

**注意**：下面的解释说明是以`Navigation2 v1.0.12`来进行的。其对应的`ROS2`版本为`Galactic`。

​	

![image-20220619212218877](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220619212218877.png)



Nav2具有下列工具：

● 加载、提供和存储地图的工具（地图服务器Map Server）

● 在地图上定位机器人的工具 (AMCL)

● 避开障碍物从A点移动到B点的路径规划工具（Nav2 Planner）

● 跟随路径过程中控制机器人的工具（Nav2 Controller）

● 将传感器数据转换为机器人世界中的成本地图表达的工具（Nav2 Costmap 2D）

● 使用行为树构建复杂机器人行为的工具（Nav2 行为树和BT Navigator）

● 发生故障时计算恢复行为的工具（Nav2 Recoveries）

● 跟随顺序航点的工具（Nav2 Waypoint Follower）

● 管理服务器生命周期的工具和看门狗(Nav2 Lifecycle Manager)

● 启用用户自定义算法和行为的插件（Nav2 Core）

<!--more-->

`Navigation2`相比较与`Navigation1`有了较大的变化。而其中最为显著的变化则是使用行为树来组合各个功能模块，以实现搭乐高积木的开发效果。



`Navigation1`中的导航功能的实现是流水线式的，主要由`move_base`来组合各个模块的功能。在`move_base`中实现了一个状态机，全局路径规划器和局部路径规划器都是以插件的形式在相应的状态中被调用的。一个到点导航的流程是，先在一个状态里调用全局路径规划器，然后进入到另外一个状态传路径给局部路径规划器，并使其执行该路径。



在`Navigation2`中，各个功能的组织则有所不同。全局路径规划功能由各个全局路径规划的插件实现，这些插件被加载到`Planer Server`中。可以通过相应的`action`向`Planer Server`请求全局规划的路径。



同理，局部路径规划器也是以插件的形式实现的。它们有统一的接口，可以被加载到`Controller Server`中。当需要执行某条路径的时候，只需要通过相应的`action`将要执行的路径发送给`Controller Server`即可。



聪明的你，应该也想到了。`Recovery Server`也是以相同的逻辑来工作的。



`Planer Server` 、` Controller Server` 和 `Recovery Server`分别实现了各自的功能，即全局路径规划，局部路径规划和恢复操作。但它们的功能是独立的，互不干扰的。而要实现一个完整的导航功能需要各个模块的配合。`BT Navigator Server`就是那个组装乐高积木的人。



每一个功能就是一颗行为树。这颗树上的节点将与`Planer Server` 、` Controller Server` 和 `Recovery Server`通信，把各个`server`的功能组合起来。比如单点导航功能，行为树上的相应节点会向`Planer Server` 请求一条规划到目标点的路径。然后另一个树上的节点与`Controller Server`通信，将规划好的路径发给`Controller Server`去执行。当然这里只是简单说明一下流程，实际使用时可以组合更多的功能。



当`BT Navigator Server`中实现了单点导航的功能，便可顺理成章的实现多点导航。`Waypoint Follower`通过连续向`BT Navigator Server`请求单点导航功能即可实现多点导航。当之前请求的单点导航完成后，再发送下一个单点导航请求。直到所有的点都已下发。



上面框图外围的`map`、`Sensor Data`和`Tf`则是共有数据。各个功能模块都可以获取到，按需取用即可。



## Navigation2初体验

1. 安装`Navigation2`

可以参考我之前的文章来操作。

[https://shoufei.blog.csdn.net/article/details/124394203](https://shoufei.blog.csdn.net/article/details/124394203)

2. 启动gazebo

```Bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

3. 启动导航

```Bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True 
```





## Navigation2相关介绍材料

官方文档：

[https://navigation.ros.org/getting_started/index.html](https://navigation.ros.org/getting_started/index.html)



`Navigation2`官方github：

其中的`issue`讨论挺值得看的

[https://github.com/ros-planning/navigation2](https://github.com/ros-planning/navigation2)



`Navigation2` 对应的论文：

[https://arxiv.org/pdf/2003.00368.pdf](https://arxiv.org/pdf/2003.00368.pdf)



`Navigation 2`系列教程：

[https://zhuanlan.zhihu.com/p/384099348](https://zhuanlan.zhihu.com/p/384099348)



目前`Navigation2`提供的插件：

[https://navigation.ros.org/plugins/index.html#plugins](https://navigation.ros.org/plugins/index.html#plugins)







---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)



















