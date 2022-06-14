---
title: ROS2 使用LifecycleNode管理节点起停等状态
categories: ROS2
tags:
  - ROS2
  - LifecycleNode
abbrlink: c2f930e6
date: 2022-05-26 15:30:16
---


有顺序的的启动节点，暂停节点，关闭节点是ROS1的一个痛点。因为在ROS1中节点启动是无序的。ROS1系统设计时并没有考虑节点启动时可能存在的互相依赖。

但在实际生产使用环境中，某些节点能正常工作是需要其他一些节点已经启动了的。

比如：需要定位功能能正常运行，前提是map_server节点已经正常加载地图文件并且已经把地图话题发布出来。



又或者你想从建图功能切到导航功能。在ROS1中，需要先将建图功能的程序杀干净。然后再启动导航程序。在ROS2中，各个节点的状态是可管理的。

在这个场景里，大可让建图程序休眠，而不用杀掉。切换功能时只需要激活相应功能的节点即可。

<!--more-->

ROS2中引入了节点生命周期管理的概念，正是为了处理上面描述的问题。这项功能的实现需要做下面两件事情。

- 继承`LifecycleNode` 来实现自己的节点
- 将节点名称注册到`Lifecycle Manager` 中，由它来管理各个节点的状态

实现一个功能通常需要一些节点互相配合来实现。这样的话，我们可以将某一个功能涉及到的节点使用一个`Lifecycle Manager` 程序来管理。从而实现了起停一项功能的效果。

### ROS2中的节点类型

ROS2中提供了两种节点类型。

- `Node() ` 是和ROS1中一样的节点基类
- `LifecycleNode()` 是可管理状态的节点基类



这里详细说说`LifecycleNode()` 节点类型。
`LifecycleNode` 类型的节点可以处于下面几种状态：
-   Unconfigured
-   Inactive
-   Active
-   Finalized

每一种状态下，可以让节点运行不同的代码。

**Unconfigured 状态**
当一个节点被创建时，它首先会进入到**Unconfigured 状态**。  
该状态下可通过执行`onConfigure()`切换到`Inactive`；  
该状态下可通过执行`onShutdown()`切换到`Finalized`。  

**Inactive 状态**
在这个状态下，将不响应话题，服务，action和参数等数据交互。处于该状态说明节点已经被配置好了。下一步激活就可以正常执行功能代码了。  
该状态下可通过执行`onCleanup()`切换到`Unconfigured`；  
该状态下可通过执行`onActivate()`切换到`active`；  
该状态下可通过执行`onShutdown()`切换到`Finalized`。  

**Active 状态**
该状态是节点的运行状态，可以接受话题消息，响应服务请求和action请求。数据接收到后进行处理然后输出结果。主要的功能代码应该在该状态下执行。  
该状态下可通过执行`onDeactivate()`切换到`inactive`；  
该状态下可通过执行`onShutdown()`切换到`Finalized`。  

**Finalized 状态**
该状态是节点待销毁前的一个状态。在该状态下执行`destroy()` 将释放节点的资源。  

下图演示了各个状态之间是如何切换的。  

![life_cycle_sm](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/life_cycle_sm.png)

图片来源于：[http://design.ros2.org/articles/node_lifecycle.html](http://design.ros2.org/articles/node_lifecycle.html)
图中，蓝色部分是表示节点状态，黄色部分是状态转换需执行的函数。

**需要注意的是，`LifecycleNode` 类型节点目前只可以在C++中使用**

从图上可以看出，`LifecycleNode` 类型节点切换状态是通过执行一系列的函数实现的。这些函数在继承`LifecycleNode` 类型节点时是需要重新实现的。

这些切换状态的函数接口声明在`/opt/ros/galactic/include/rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp`文件中。  
主要有下面几个：
```c++
  
/// Callback function for configure transition

/*

* \return true by default

*/

RCLCPP_LIFECYCLE_PUBLIC

virtual CallbackReturn

on_configure(const State & previous_state);

  

/// Callback function for cleanup transition

/*

* \return true by default

*/

RCLCPP_LIFECYCLE_PUBLIC

virtual CallbackReturn

on_cleanup(const State & previous_state);

  

/// Callback function for shutdown transition

/*

* \return true by default

*/

RCLCPP_LIFECYCLE_PUBLIC

virtual CallbackReturn

on_shutdown(const State & previous_state);

  

/// Callback function for activate transition

/*

* \return true by default

*/

RCLCPP_LIFECYCLE_PUBLIC

virtual CallbackReturn

on_activate(const State & previous_state);

  

/// Callback function for deactivate transition

/*

* \return true by default

*/

RCLCPP_LIFECYCLE_PUBLIC

virtual CallbackReturn

on_deactivate(const State & previous_state);

  

/// Callback function for errorneous transition

/*

* \return false by default

*/

RCLCPP_LIFECYCLE_PUBLIC

virtual CallbackReturn

on_error(const State & previous_state);
```

每一个状态转换函数可以返回三种状态：
```c++
enum class CallbackReturn : uint8_t

{

SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS,

FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE,

ERROR = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR

};
```

在任何状态下，执行状态转换函数返回错误(`ERROR`)时，节点都会执行`on_error()`。如果执行成功则节点转换到`Unconfigured`状态。如果执行失败则节点转换到`Finalized`状态。

每一个状态转移函数都有需要实现的功能。
**onConfigure()**
顾名思义，该函数主要负责配置好节点，包括初始化节点内要使用的资源，读取参数，新建话题发布器订阅器，服务，action等等。

`onConfigure`运行成功，节点将从`Unconfigured`切换到`Inactive`状态。如果运行失败则仍然处于`Unconfigured`状态。如果运行返回错误则进行错误处理（即`onError()`)。

**onCleanup()**
主要用于清除节点的状态。使节点内各状态数据恢复到节点刚刚创建的时候。

`onCleanup`运行成功，节点将从`Inactive`切换到`Unconfigured`状态。如果运行返回错误则进行错误处理（即`onError()`)。

**onActivate()**
在该函数里一般做一些节点功能运行前的最后准备工作。比如激活话题订阅器发布器等等。

`onActivate`运行成功，节点将从`Inactive`切换到`Active`状态。如果运行返回错误则进行错误处理（即`onError()`)。

**onDeactivate()**
在该函数里执行的操作一般与`onActivate()`相反。比如复位话题订阅器发布器等等。

`onDeactivate`运行成功，节点将从`Active`切换到`Inactive`状态。如果运行返回错误则进行错误处理（即`onError()`)。

**onShutdown()**
在这里主要执行节点销毁前的一些操作。除了`Finalized`状态外，其他任何状态下都可以运行该函数使节点状态切换至`Finalized`状态。

`onShutdown`运行成功，节点将目前状态切换到`Finalized`状态。如果运行返回错误则进行错误处理（即`onError()`)。

**onError()**
任何其他状态转移函数执行错误都将执行这个函数。在这里执行一些异常的应对策略。

`onError`运行成功，节点将切换到`Unconfigured`状态。如果运行返回错误则进入到`Finalized`状态。


### 如何建立一个`LifecycleNode` 节点
下面是一个简单的示例。
```c++
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ManagedScan : public rclcpp_lifecycle::LifecycleNode
{
public:

  explicit ManagedScan(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {}

  void
  publish()
  {
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

    if (!pub_->is_activated()) {
      RCLCPP_INFO(
        get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    } else {
      RCLCPP_INFO(
        get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
    }

    pub_->publish(std::move(msg));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("managed_scan", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&ManagedScan::publish, this));

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    pub_->on_activate();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    std::this_thread::sleep_for(2s);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    pub_->on_deactivate();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {

    timer_.reset();
    pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {

    timer_.reset();    
    pub_.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
    
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &)
  {

    RCUTILS_LOG_INFO_NAMED(get_name(), "something went wrong!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

  std::shared_ptr<rclcpp::TimerBase> timer_;
};


int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<ManagedScan> lc_node =
    std::make_shared<ManagedScan>("managed_scan_node");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
```

通过下面的方式获取完整的`lifecycle_node_demo`工程。
```bash
git clone https://gitee.com/shoufei/lifecycle_node_demo.git
```

需要注意的是，LifecycleNode 类型节点中的话题发布器需要按下面的方式定义。
```c++
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
```
话题订阅器则与普通Node类型节点一样。

编译并source了该工程后就可以用下面的命令启动该示例。
```bash
ros2 launch lifecycle_node_demo lifecycle_demo.launch.py
```

用` ros2 topic list`命令查看，可以发现有下面这个话题。
```bash
/lifecycle_node_demo_node/transition_event
```
但是没有代码中定义的`managed_scan`话题。这是因为节点启动后并没有执行`on_configure`函数。它还处于`Unconfigured`状态。而发布器的定义是在`on_configure`函数中的。

通过下面的命令可获取节点状态
```bash
ros2 lifecycle get /lifecycle_node_demo_node
```
此时它返回
```bash
unconfigured [1]
```

当我们打印`/lifecycle_node_demo_node/transition_event`话题内容时，发现并没有任何数据。那是因为目前并没有发生状态转换。

使用下面的命令来设置节点状态
```bash
ros2 lifecycle set /lifecycle_node_demo_node configure
```
可以设置的状态有下面几个
- configure
- cleanup 
- activate 
- deactivate
- shutdown 


转换状态成功后将打印
```bash
Transitioning successful
```

`/lifecycle_node_demo_node/transition_event`话题也打印了下面的信息
```bash
timestamp: 0
transition:
  id: 0
  label: ''
start_state:
  id: 1
  label: unconfigured
goal_state:
  id: 10
  label: configuring
---
timestamp: 0
transition:
  id: 0
  label: ''
start_state:
  id: 10
  label: configuring
goal_state:
  id: 2
  label: inactive
---


```

`managed_scan`话题也能用`ros2 topic list`看到了。

如果需要重新启动一个节点，可以按下面的步骤进行操作：
1. 将节点切换到`inactive`状态
2. 然后将节点切换到`unconfigured`状态
3. 接着将节点切换到`inactive`状态
4. 最后再切换到`active`状态

### 如何管理`LifecycleNode` 节点的状态

`LifecycleNode` 节点提供了切换状态的服务，所以可以通过外部程序通过服务请求的方式来管理`LifecycleNode` 节点的状态切换。

在Navigation2中的`nav2_lifecycle_manager`功能包实现了对`LifecycleNode` 节点的管理。

以Navigation2中的navigation_launch.py文件为例来分析一下`nav2_lifecycle_manager`功能包如何管理多个节点。

```python
# Copyright (c) 2018 Intel Corporation

#

# Licensed under the Apache License, Version 2.0 (the "License");

# you may not use this file except in compliance with the License.

# You may obtain a copy of the License at

#

# http://www.apache.org/licenses/LICENSE-2.0

#

# Unless required by applicable law or agreed to in writing, software

# distributed under the License is distributed on an "AS IS" BASIS,

# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

# See the License for the specific language governing permissions and

# limitations under the License.

  

import os

  

from ament_index_python.packages import get_package_share_directory

  

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

  
  

def generate_launch_description():

# Get the launch directory

bringup_dir = get_package_share_directory('nav2_bringup')

  

namespace = LaunchConfiguration('namespace')

use_sim_time = LaunchConfiguration('use_sim_time')

autostart = LaunchConfiguration('autostart')

params_file = LaunchConfiguration('params_file')

  

lifecycle_nodes = ['controller_server',

'planner_server',

'recoveries_server',

'bt_navigator',

'waypoint_follower']

  

# Map fully qualified names to relative ones so the node's namespace can be prepended.

# In case of the transforms (tf), currently, there doesn't seem to be a better alternative

# https://github.com/ros/geometry2/issues/32

# https://github.com/ros/robot_state_publisher/pull/30

# TODO(orduno) Substitute with `PushNodeRemapping`

# https://github.com/ros2/launch_ros/issues/56

remappings = [('/tf', 'tf'),

('/tf_static', 'tf_static')]

  

# Create our own temporary YAML files that include substitutions

param_substitutions = {

'use_sim_time': use_sim_time,

'autostart': autostart}

  

configured_params = RewrittenYaml(

source_file=params_file,

root_key=namespace,

param_rewrites=param_substitutions,

convert_types=True)

  

return LaunchDescription([

# Set env var to print messages to stdout immediately

SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

  

DeclareLaunchArgument(

'namespace', default_value='',

description='Top-level namespace'),

  

DeclareLaunchArgument(

'use_sim_time', default_value='false',

description='Use simulation (Gazebo) clock if true'),

  

DeclareLaunchArgument(

'autostart', default_value='true',

description='Automatically startup the nav2 stack'),

  

DeclareLaunchArgument(

'params_file',

default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),

description='Full path to the ROS2 parameters file to use'),

  

Node(

package='nav2_controller',

executable='controller_server',

output='screen',

parameters=[configured_params],

remappings=remappings),

  

Node(

package='nav2_planner',

executable='planner_server',

name='planner_server',

output='screen',

parameters=[configured_params],

remappings=remappings),

  

Node(

package='nav2_recoveries',

executable='recoveries_server',

name='recoveries_server',

output='screen',

parameters=[configured_params],

remappings=remappings),

  

Node(

package='nav2_bt_navigator',

executable='bt_navigator',

name='bt_navigator',

output='screen',

parameters=[configured_params],

remappings=remappings),

  

Node(

package='nav2_waypoint_follower',

executable='waypoint_follower',

name='waypoint_follower',

output='screen',

parameters=[configured_params],

remappings=remappings),

  

Node(

package='nav2_lifecycle_manager',

executable='lifecycle_manager',

name='lifecycle_manager_navigation',

output='screen',

parameters=[{'use_sim_time': use_sim_time},

{'autostart': autostart},

{'node_names': lifecycle_nodes}]),

  

])
```

其中`lifecycle_nodes`变量定义了需要管理的节点。**注意，这些节点都是继承于`LifecycleNode`。**
```python
lifecycle_nodes = ['controller_server',

'planner_server',

'recoveries_server',

'bt_navigator',

'waypoint_follower']
```

然后将需要管理的节点名称传入`nav2_lifecycle_manager`包。其中`autostart`参数决定了是否自动启动这些节点并把状态切换到`Active`状态。

参考：
[http://design.ros2.org/articles/node_lifecycle.html](http://design.ros2.org/articles/node_lifecycle.html)
