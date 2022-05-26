---
title: [ROS2] LifecycleNode 命令行工具 #文章页面上的显示名称，一般是中文
date: 2022-05-26 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2,LifecycleNode] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: LifecycleNode命令行工具
---


使用`ros2 lifecycle --h`命令可以看到`lifecycle`相关的命令有哪些。

### 获取LifecycleNode节点的状态

```
ros2 lifecycle get /lifecycle_node_demo_node
```
其中`/lifecycle_node_demo_node`为节点名称


### 设置LifecycleNode节点的状态
```
ros2 lifecycle set /lifecycle_node_demo_node configure

```
其中`/lifecycle_node_demo_node`为节点名称

可以设置的状态有下面几个
- configure
- cleanup 
- activate 
- deactivate
- shutdown 

### 查看系统中有哪些LifecycleNode节点

```
ros2 lifecycle nodes
```

### 查看LifecycleNode节点的所有可行的转换
```
ros2 lifecycle list lifecycle_node_demo_node -a
```
其中`/lifecycle_node_demo_node`为节点名称

显示基于当前状态的可行切换函数
```
ros2 lifecycle list lifecycle_node_demo_node
```
