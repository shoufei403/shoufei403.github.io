---
title: ROS2基础之参数和动作 #文章页面上的显示名称，一般是中文
date: 2022-05-26 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2] #文章标签，可空，多标签请用格式，注意:后面有个空格
# description: ROS2参数和动作的使用
---

文本中用于说明的完整示例代码请在下面的仓库中查阅。

[https://gitee.com/shoufei/ros2_galactic_turorials.git](https://gitee.com/shoufei/ros2_galactic_turorials.git)

[https://github.com/shoufei403/ros2_galactic_tutorials.git](https://github.com/shoufei403/ros2_galactic_tutorials.git)

## 参数

### 理解参数的概念

节点的参数通常维护到`yaml`文件中。节点启动时通常可以加载参数文件，然后读取参数的内容。而且在`ROS`中参数是可以动态配置的。

更多细节参考官方文档：  

[https://docs.ros.org/en/galactic/Tutorials/Parameters/Understanding-ROS2-Parameters.html](https://docs.ros.org/en/galactic/Tutorials/Parameters/Understanding-ROS2-Parameters.html)



### 给节点设置参数的三种方式

1. **让节点加载参数文件**

以命令行方式加载参数文件

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

示例：

```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
```
<!--more-->
2. **以`launch`文件的形式启动节点并加载配置文件**

```python
tb3_param_dir = LaunchConfiguration(
    'tb3_param_dir',
    default=os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'param',
        TURTLEBOT3_MODEL + '.yaml'))
        
Node(
    package='turtlebot3_node',
    executable='turtlebot3_ros',
    parameters=[tb3_param_dir],
    arguments=['-i', usb_port],
    output='screen')
```



3. **命令行工具设置参数**

罗列参数

```bash
ros2 param list
```



获取参数值

```bash
ros2 param get <node_name> <parameter_name>
```

示例：

```bash
ros2 param get /turtlesim background_g
```

设置参数值

```bash
ros2 param set <node_name> <parameter_name> <value>
```

示例：

```bash
ros2 param set /turtlesim background_r 150
```

保存当前运行节点的参数为文件

```bash
ros2 param dump <node_name>
```

示例：

```bash
ros2 param dump /turtlesim
```

加载节点参数

```bash
ros2 param load <node_name> <parameter_file>
```

示例：

```bash
ros2 param load /turtlesim ./turtlesim.yaml
```

4. **动态参数配置（GUI）**

动态参数配置主要通过`rqt`来实现。

`rqt->Plugins->Configuration->Dynamic Reconfigure`

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20220417105449.png)



修改了参数后会立即生效。具体的实现流程是，参数变化后产生参数事件，触发回调函数。在回调函数中将新的参数赋值给变量。

以下为GUI动态参数配置对应执行的代码

```c++
// Subscription for parameter change
rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    
// Setup callback for changes to parameters.
parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
  node->get_node_base_interface(),
  node->get_node_topics_interface(),
  node->get_node_graph_interface(),
  node->get_node_services_interface());

//register callback function
parameter_event_sub_ = parameters_client_->on_parameter_event(
  std::bind(&TurtlesimController::on_parameter_event_callback, this, std::placeholders::_1));

//callback function
void TurtlesimController::on_parameter_event_callback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::lock_guard<std::mutex> l(config_mutex_);
  
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      // Trajectory
      if (name == "walk_distance") {
        walk_distance_ = value.double_value;
        RCLCPP_INFO(this->get_logger(), "update walk_distance: %.3f ", walk_distance_);
      }
    }
    else if(type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    {
      if (name == "execute_frequency") {
        execute_frequency_ = value.integer_value;//不会改变定时器执行频率，仅作为演示用
        RCLCPP_INFO(this->get_logger(), "update execute_frequency: %d ", execute_frequency_);
      }
    }
    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    {
      if (name == "print_execute_duration") {
        print_execute_duration_ = value.bool_value;
        RCLCPP_INFO(this->get_logger(), "update print_execute_duration: %d ",print_execute_duration_);
      }
    }
    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    {
      if (name == "show_str") {
        show_str_ = value.string_value;
        RCLCPP_INFO(this->get_logger(), "update show_str: %s ",show_str_.c_str());
      }
    }
  }
}
```

### 如何编写可参数配置的节点

参考turtlesim_controller包中的代码

[https://gitee.com/shoufei/ros2_galactic_turorials.git](https://gitee.com/shoufei/ros2_galactic_turorials.git)



## 动作 Action

### Action的概念

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/Action-SingleActionClient.gif)



`Action`可以认为是两个服务和一个消息通讯的组合。两个服务分别是发送目标请求的服务和请求执行结果的服务。其中的消息通讯主要是用于反馈数据的。`Action`主要用在任务需要一段时间才能执行完成的场景。这里举一个例子来帮助理解：你和你女朋友准备玩一个简单的游戏。游戏很简单，就是你请求你女朋友向前跑一百米。然后你女朋友开始跑，然后跑完告诉你结果。现在我们来详细描述一下这个游戏过程。



首先你向你女朋友提出向前跑一百米的请求（**对应发出`Goal`请求**），你女朋友接受到了这个请求，但是觉得你不够诚恳所以拒绝了你（**因请求数据不合理action可拒绝goal请求**）。你调整了自己的语气再次向你女朋友发出了请求。这次你女朋友答应了（**接受了`Action`的`goal`**）。你很高兴，告诉你女朋友等她跑完了要通知自己。然后你女朋友就开始跑了。为了告诉你她跑的进度，你女朋友在跑的过程中主动告诉你她跑了多远了，还有多远就跑完了（**对应`Action`的数据反馈**）。当你女朋友跑完一百米后，她通知你她已经跑完了（**反馈执行结果**）。



更多细节说明请查看官方文档：

[https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Actions.html](https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Actions.html)



### 运行Action示例

启动小乌龟

```bash
ros2 run turtlesim turtlesim_node
```

启动手动控制小乌龟的节点

```bash
ros2 run turtlesim turtle_teleop_key
```

Action通信框图

```bash
ros2 run rqt_graph rqt_graph
```

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20220417110034.png)



### Action 示例代码

请查看`examples/rclcpp`和`examples/rclpy`中的示例代码。



### 自定义Action

[https://docs.ros.org/en/galactic/Tutorials/Actions/Creating-an-Action.html](https://docs.ros.org/en/galactic/Tutorials/Actions/Creating-an-Action.html)

查看示例代码中的`tutorial_interfaces`包。



## 节点通讯常用的ROS2命令行工具

### 显示节点通讯方式及其类型

第一种方式，查看特定node具备的通讯方式

```bash
ros2 node list
ros2 node info /turtlesim
➜  src ros2 node info /turtlesim
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```



第二种方式，查看整个系统中的通讯方式

```bash
➜ ros2 topic list -t  #前面为通讯方式的名称，中括号内为通讯类型                 
1639005245.255618 [0]       ros2: using network interface wlp109s0 (udp/192.168.31.206) selected arbitrarily from: wlp109s0, br-282e04a6a7cf, docker0
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
➜ ros2 service list -t    #前面为通讯方式的名称，中括号内为通讯类型              
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/teleop_turtle/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/teleop_turtle/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/teleop_turtle/get_parameters [rcl_interfaces/srv/GetParameters]
/teleop_turtle/list_parameters [rcl_interfaces/srv/ListParameters]
/teleop_turtle/set_parameters [rcl_interfaces/srv/SetParameters]
/teleop_turtle/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
➜ ros2 action list -t    #前面为通讯方式的名称，中括号内为通讯类型
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

在代码编写时，常常要获取节点的通信类型。以类型`[turtlesim/action/RotateAbsolute]`为例，在代码中以该类型声明变量的方式为`turtlesim::action::RotateAbsolute `。

### 显示类型的数据结构

命令行工具为：

```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

输出内容：

```SQL
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

在代码中，对通信类型的变量进行赋值时也常常会要查询类型的数据结构。

### 手动发起一次通信

**话题**

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
示例：
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

**服务**

```bash
ros2 service call <service_name> <service_type> <arguments>
示例：
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

**动作**

```bash
ros2 action send_goal <action_name> <action_type> <values>
示例：
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

使用`ROS2`命令行手动发起一次通信常常用于做测试。

---



**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
