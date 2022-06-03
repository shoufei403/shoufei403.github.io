---
title: ROS2基础之话题与服务 #文章页面上的显示名称，一般是中文
date: 2022-05-26 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2] #文章标签，可空，多标签请用格式，注意:后面有个空格
# description: ROS2话题与服务的使用
---



本文介绍`ROS2`中话题和服务的内容。并配合示例代码来实践以便加深理解。

## 安装依赖

安装相应的工具：

`colcon` 编译工具安装

```bash
sudo apt install python3-colcon-common-extensions git
```

设置`colcon_cd`，方便用`colcon_cd 包名` 快速打开到包的目录。需要在工作空间目录下使用，其他路径下使用会出现卡住的情况。

```Bash
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc

#设置colcon_cd打开的根目录，根据自己的实际情况设置
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
```

设置`colcon`参数自动补全功能

```Bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

`ROS2`参数自动补全工具（`zsh`不支持`ROS2`参数自动补全，`bash`可以）

```bash
sudo apt-get install python3-argcomplete
```
<!--more-->


会用到的几个命令：

编译整个工作空间

```bash
colcon build --symlink-install
```

编译单个功能包（包名需要是package.xml文件中name标签中的名字）

```bash
colcon build --symlink-install --packages-select 包名
```

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/8c7549c59627c09ed3a618bf47841371.png)

source环境

```Bash
#source ros2 系统层环境变量
source /opt/ros/galactic/setup.bash
#source ros2 工作空间层环境变量（galactic_ws为用户工作空间的名称）
source ~/galactic_ws/install/setup.bash
```

创建功能包

```bash
ros2 pkg create test_pkg --node-name test_cpp --dependencies rclcpp std_msgs
```

运行节点

```bash
ros2 run 包名 执行文件名
#示例：ros2 run turtlesim turtlesim_node
```

安装ros包的命令

```bash
sudo apt-get install ros-<distro>-<package>
#示例(安装gazebo相关的包)：sudo apt-get install ros-galactic-gazebo-*
```

`ROS2 `命令行工具速查表，请在公众号《**首飞**》中回复“cli”获取。



## **ROS2 工作空间**

工作空间用于存放ros功能包。

#### **ROS2系统工作空间**

目录如下：  

```bash
/opt/ros/galactic/
```

用命令安装的ros功能包都会在这个目录下。

```Bash
source /opt/ros/galactic/setup.bash
```

`source `该路径下的`setup.bash`可以获取`ROS2`系统维护的环境变量。相当于告诉电脑在哪里去找对应的工具和执行文件。比如`ros2`的命令行工具。



#### **ROS2用户工作空间**

例如：`~/galactic_ws`



#### **创建用户工作空间**

```Bash
mkdir -p galactic_ws/src
```

实际上就是创建一系列的目录。ros功能包统一放到src目录下。



## **ROS2节点**

一个节点只实现一个单独的功能（模块化）。实现节点功能的代码会组织成一个功能包（package）。

下面是一个典型的机器人系统框架图。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/2f59da57e8527bcd7da2b79b16230812.png)



一个工作空间下可以有多个功能包，一个功能包可以有多个节点存在

#### **创建ROS2功能包**

在`src`目录下运行下面命令，创建一个包名为`test_pkg`，执行文件为`test_cpp`，并且依赖`rclcpp std_msgs`等功能包的`ROS2`功能包。

```bash
ros2 pkg create test_pkg --node-name test_cpp --dependencies rclcpp std_msgs
```

安装tree命令

```Bash
sudo apt-get install tree
```

在galactic_ws下运行`tree`查看生成了哪里文件。

在galactic_ws下运行`code .` 查看生成的文件内容。



#### **编译ROS2功能包**

在galactic_ws目录下运行

```Bash
colcon build --symlink-install
```

`--symlink-install `指对于解释性的代码，如python，修改了源码后无需编译即可生效。`ros2 run `会执行修改后的内容。因为安装文件是相应源码文件的软链接。对于编译性的源码则无效。比如修改C++源码后需要重新编译才能生效。

在galactic_ws下运行`tree` ，查看多了哪些文件。

编译命令的参考资料

https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html

https://colcon.readthedocs.io/en/released/user/installation.html



#### **运行ROS2节点**

```bash
source install/setup.bash
ros2 run test_pkg test_cpp
```

可以看到打印了hello world。

## **下载示例代码**

在`galactic_ws`工作空间下的`src`目录下运行下面命令获取示例代码。

```bash
git clone https://gitee.com/shoufei/ros2_galactic_turorials.git
```



按照仓库的`readme.md`文件进行编译



## **节点间的通讯**

有下面4种方式：

- 话题-topics

- 服务-services

- 动作-Action

- 参数-parameters

本篇文章讲前两个：话题和服务。



**看到这里需要提醒一下**，如果已经按照上面的说明编译好了示例代码，下面就可以试着运行一下下面的示例程序看看效果了。



#### **话题-topics**

##### **话题概念 （参考官方文档）**

https://docs.ros.org/en/galactic/Tutorials/Topics/Understanding-ROS2-Topics.html

话题通讯接口是发布和订阅模型。一个节点程序里可以有多个发布器，也可以有多个接受器。发布器数据时，只管发布不管谁会接受。同理，接收器接收数据时，也只管接收对于的数据，不管到底是谁发的。那它们怎么对应起来呢？主要是通过数据的名称（即话题名称）来对应起来。

##### **运行话题示例**

运行订阅话题的节点

```Bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

运行发布话题的节点

```Bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

查看节点图

```Bash
rqt
```

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/72378de6a4f955ad74b55fd0d649e7b3.png)



下面的路径里可以查看`ROS2`系统已经定义好的标准消息类型：

```Bash
/opt/ros/galactic/include/std_msgs/msg
```

他们是一系列自动生成的文件。



##### **自定义话题消息类型**

https://docs.ros.org/en/galactic/Tutorials/Custom-ROS2-Interfaces.html

自定义消息类型和自定义一个`C++`的结构体很像。但使用的类型需要是之前已经定义好的。



**运行示例**

```bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function_with_custom_msg
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function_with_custom_msg
```



##### **定义发布器和订阅器的示例**

```c++
vel_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
turtlesim_pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
    "turtle1/pose", 1, std::bind(&TurtlesimController::turtlesim_pose_callback, this, std::placeholders::_1));
```





#### **服务-services**

##### **服务概念 （参考官方文档）**

https://docs.ros.org/en/galactic/Tutorials/Services/Understanding-ROS2-Services.html



可以通过`rqt`中的`service caller`插件来发送服务请求。

##### **运行服务示例**

运行服务端的节点

```bash
ros2 run examples_rclcpp_minimal_service service_main
```

运行客户端的节点

```bash
ros2 run examples_rclcpp_minimal_client client_main
```



在下面的路径下可以查看`ROS2`系统定义的服务类型

```Groovy
/opt/ros/galactic/include/std_srvs/srv
```

##### **自定义服务类型**

https://docs.ros.org/en/galactic/Tutorials/Custom-ROS2-Interfaces.html



**运行示例**

```bash
ros2 run examples_rclcpp_minimal_service service_main_custom_srv
ros2 run examples_rclcpp_minimal_client client_main_custom_srv 2 3 4
```

##### **创建服务的示例**

```c++
server_ = this->create_service<TurtleCmdMode>("change_turtle_control_mode", 
   std::bind(&TurtlesimController::handle_turtle_control_mode_service, this, std::placeholders::_1, std::placeholders::_2));
```





来张图总结一下

![](https://docs.ros.org/en/galactic/_images/Nodes-TopicandService.gif)

自定义消息和服务的注意事项：

1. 消息和服务文件中，变量名需要是小写的，写成大写不能编译过。
2. 消息和服务文件的命名需要是每个单词开头字母为大写。如下所示：

```C%2B%2B
SensorData.msg
```

1. 编译成功后，使用时需要包含对应接口的头文件。头文件的形式为：

```C%2B%2B
#include <包名/msg/sensor_data.hpp>
```

注意与消息或服务文件名字的对应关系。





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)

