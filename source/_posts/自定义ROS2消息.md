---
title: 自定义ROS2消息 #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: 自定义ROS2消息
---

这篇文章着重介绍自定义`ROS2`类型数据时，我们可以使用哪些基础类型。这也是我之前困惑的一个问题。所以这里简单汇总说明一下。关于消息、服务和动作接口的使用说明，请查看本次推送的第一篇推文。



## 自定义msg示例

定义`msg`数据和在`C++`中定义一个结构体是一样的。只是类型名称有些区别。然后不需要加分号。

```c++
Header header
float64 circle_x
float64 circle_y
float32[] ranges
```



## 自定义service数据示例

`service`数据包含两个部分：请求和应答。这两部分用`---`隔开。两部分数据可分别定义，并且可以为空类型（`std_msgs/Empty`）。

```c++
# Request 
geometry_msgs/PoseStamped goal
geometry_msgs/PoseStamped start
string planner_id
bool use_start # If true, use current robot pose as path start, if false, use start above instead
---
# Respond
nav_msgs/Path path
```



## 自定义action示例

`action`数据则包含三部分：请求数据、返回结果和反馈数据。其中请求数据和返回结果分别只会传输一次。而反馈数据则可持续传输一直到整个`action`服务结束。

带常量的形式

```

# Request 
int8 FOLLOW_RIGHT = -1 
int8 FOLLOW_LEFT = 1 

int8 follow_side 
builtin_interfaces/Duration max_runtime 
--- 
# Result 
builtin_interfaces/Duration runtime 
--- 
# Feedback 
bool engaged
```



## 可以设定的数据类型有哪些

- 有符号整型

  ```c++
  int8
  int16
  int32
  int64  
  ```

- 无符号整型

  ```c++
  uint8
  uint16
  uint32
  uint64
  ```

- 常量

  ```c++
  int8 FOLLOW_RIGHT = -1 
  int8 FOLLOW_LEFT = 1 
  ```

  

- 字符串

  ```c++
  string
  ```

  

- 浮点数

  ```c++
  float32
  float64
  ```

- 布尔

  ```c++
  bool
  ```

- ROS2 预设数据类型

  消息头部

  ```c++
  Header 
  std_msgs/Header
  ```

  路径类型
  ```c++
  nav_msgs/Path
  ```

  pose类型

  ```c++
  geometry_msgs/PoseStamped[] 
  geometry_msgs/PoseStamped 
  geometry_msgs/Pose
  geometry_msgs/Point32
  geometry_msgs/Vector3
  geometry_msgs/Point
  ```

  空类型

  ```c++
  std_msgs/Empty
  ```

  时间类型

  ```c++
  builtin_interfaces/Duration
  builtin_interfaces/Time
  ```

系统中安装的msgs都可以在自定义消息时使用

![](/home/kevin/kevin_blogs/TMP_images/20220512130058.png)

这里展示一下标准类型

![](/home/kevin/kevin_blogs/TMP_images/20220512130409.png)

- 数组

  ```c++
  float32[]
  ```

  

## 自定义数据的示例代码

示例代码请查看下面的网址：

[https://gitee.com/shoufei/ros2_galactic_turorials/tree/master/tutorial_interfaces](https://gitee.com/shoufei/ros2_galactic_turorials/tree/master/tutorial_interfaces)

下载完整示例代码包：

```bash
git clone https://gitee.com/shoufei/ros2_galactic_turorials.git
```



下面是代码包中`readme`文件的内容，编写代码时可简单查阅一下。

### 类型文件的命名方式

大写字母开头，多个单词命名则每个单词首字母为大写。



### 类型文件的定义 

浮点型： float32
整型：   int32
字符串： string
整型数组： int32[]
浮点型数组： float32[]



### 本包生成的通讯类型
action:  

action类型

```
tutorial_interfaces/action/GoLine 
```


在代码中声明变量：  

```
tutorial_interfaces::action::GoLine
```
使用时需要包含的头文件  

```
#include <tutorial_interfaces/action/go_line.hpp>
```

查看类型的数据内容

```
ros2 interface show tutorial_interfaces/action/GoLine
```



service:

service类型

```
tutorial_interfaces/srv/TurtleCmdMode
```

在代码中声明变量：

```
tutorial_interfaces::srv::TurtleCmdMode
```

使用时需要包含的头文件 

```
#include <tutorial_interfaces/srv/turtle_cmd_mode.hpp>
```

查看类型的数据内容

```
ros2 interface show tutorial_interfaces/srv/TurtleCmdMode
```



topic:

topic类型

```
tutorial_interfaces/msg/Num
```

在代码中声明变量：

```
tutorial_interfaces::msg::Num
```

使用时需要包含的头文件 

```
#include <tutorial_interfaces/msg/num.hpp>
```

查看类型的数据内容

```
ros2 interface show tutorial_interfaces/msg/Num
```





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
