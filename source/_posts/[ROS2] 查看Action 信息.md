---
title: ROS2 查看Action信息 #文章页面上的显示名称，一般是中文
date: 2022-05-26 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2,Action] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: 查看Action 信息
---

查看Action 信息的常用命令
Action主要用于长时间运行的任务。它们由三部分组成：目标、反馈和结果。

查看action列表
```
ros2 action list
```

查看action列表和类型
```
ros2 action list -t
```

查看action信息
```
ros2 action info <action_name>
```

显示action
```
ros2 interface show <action_name>
```

action send goal
```
ros2 action send_goal <action_name> <action_type> <values>
```



关于action，topic，service，params等通讯接口的查询命令，我准备了一个文档。可在公众号《**首飞**》中回复”cli“获取。
