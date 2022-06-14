---
title: ROS2 查看Action信息
categories: ROS2
tags:
  - ROS2
  - Action
abbrlink: c1dd3084
date: 2022-05-26 15:30:16
---

查看Action 信息的常用命令
Action主要用于长时间运行的任务。它们由三部分组成：目标、反馈和结果。

查看action列表
```bash
ros2 action list
```

查看action列表和类型
```bash
ros2 action list -t
```

查看action信息
```bash
ros2 action info <action_name>
```

显示action
```bash
ros2 interface show <action_name>
```

action send goal
```bash
ros2 action send_goal <action_name> <action_type> <values>
```
<!--more-->


关于action，topic，service，params等通讯接口的查询命令，我准备了一个文档。可在公众号《**首飞**》中回复”cli“获取。
