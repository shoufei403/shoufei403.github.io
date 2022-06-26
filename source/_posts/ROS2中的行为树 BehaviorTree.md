---
title: ROS2中的行为树 BehaviorTree
categories: ROS2
tags:
  - ROS2
  - BehaviorTree
abbrlink: a749d57d
date: 2022-06-26 20:28:17
---

`BehaviorTree.CPP`是一个开源的`C++`行为树库。在游戏领域，行为树已经比较流行了。主要用于维护游戏角色的各种动作和状态。但在机器人领域还很少使用的。`Navigation2`中引入了行为树来组织机器人的工作流程和动作执行。



行为树是树状的结构，它的逻辑流程是由`xml`文件描述的。我们可以用其配套的工具`Groot`来可视化行为树。如下图所示：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626161939314.png)



<!--more-->

行为树本身并不具体实现机器人的执行内容，它只负责将执行内容进行编排。以`Navigation2`为例，具体的执行内容实现是放在各个`server`中的。行为树上的节点与`server`进行通信，请求具体的执行内容，然后获得反馈。根据反馈结果又可以请求另外的执行内容。这些不同执行内容间的跳转就是由行为树控制的。

![image-20220619212218877](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220619212218877.png)





## 行为树与状态机的对比

另一种比较常见的组织机器人行为的方式是状态机。`ROS1`中的`move_base`就是基于状态机的。它与行为树最显著的区别是状态与执行内容是绑定在一起的。当执行内容需要在多个状态中执行时，各个状态下都需要放置执行内容的逻辑。当业务逻辑代码分散在各处时就不太好维护了，特别是对于复杂的机器人系统。



一个好的软件有如下特点（引用[官方文档](https://www.behaviortree.dev/)的一段话）：

A "good" software architecture should have the following characteristics:

- Modularity.

- Reusability of componens.

- Composability.

- Good separation of concerns.

Main advantages :

  - They are intrinsically Hierarchical
  - Their graphical representation has a semantic meaning
  - They are more expressive



Behavior Tree VS FSM(Finite State Machines)：

The business logic becomes "spread" in many locations and it is hard for the developer to reason about it and to debug errors in the control flow.
业务逻辑变得 "分散 "在许多地方，开发人员很难对其进行推理并调试控制流中的错误。

To achieve strong separation of concerns it is better to centralize the business logic in a single location.
为了实现强大的关注点分离，最好是将业务逻辑集中在一个地方。



## 行为树初体验

下面是官方的演示视频，可以感受一下。

<iframe 
    width="800" 
    height="450" 
    src="//player.bilibili.com/player.html?aid=555302822&bvid=BV1Rv4y1M72W&cid=756163364&page=1"
    frameborder="0" 
    allowfullscreen>
</iframe>


## 编译安装Groot

编译安装`behaviortree-cpp-v3`

```Bash
sudo apt-get install libzmq3-dev libboost-dev
git clone https://ghproxy.com/https://github.com/BehaviorTree/BehaviorTree.CPP
cd BehaviorTree.CPP
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

安装依赖

```Bash
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
```

编译`Groot`

```Bash
   git clone https://ghproxy.com/https://github.com/BehaviorTree/Groot.git
   cd Groot
   git submodule update --init --recursive
   mkdir build
   cd build
   cmake ..
   make
```

安装`Groot`

```Bash
sudo vim /etc/ld.so.conf  #添加动态库链接地址
添加 /usr/local/lib/ 到/etc/ld.so.conf文件里
sudo ldconfig

sudo mv /usr/local/lib/groot/Groot /usr/local/bin/

然后就可以直接 用Groot命令打开 Groot
```



该库中包含了一些示例程序。当按照上面的步骤编译好后，就可以在`build/examples`中看到示例程序的执行文件。

可以运行一个看看。这里默认你已经进入到了`build/examples`目录中。

```bash
./t05_cross_door loop
```

```bash
Groot
```



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626161308145.png)



关于行为树的概念，建议看一下官方文档：

BehaviorTree.CPP  

[https://github.com/BehaviorTree/BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)

BehaviorTree官方文档：  

[https://www.behaviortree.dev/bt_basics/](https://www.behaviortree.dev/bt_basics/)



## 使用Groot

**使用`Groot`编辑行为树**

1. 打开`Groot`

```bash
Groot
```

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626152616326.png)





2. 加载自定义的行为树节点。

下图是加载了`Navigation2`中自定义的叶子节点，即其中的蓝色部分，黑色的是`BehaviorTree.CPP`库里自带的。加载的文件是`nav2_behavior_tree/nav2_tree_nodes.xml`。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626152929136.png)



3. 加载一颗行为树

上面加载的是一些可用的节点。当我们把这些可用的节点组合起来形成一颗树时，就可以实现各式各样的功能。

下图是`Navigation2`中，实现单点导航的一颗行为树。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626153940539.png)



在`Navigation2`中，描述行为树的`xml`文件存放在`nav2_bt_navigator/behavior_trees`目录下。

你可以从左侧拖动你需要的节点到右侧，然后修改节点的参数，再将其连接到树中。完成修改后保存就可以被`Navigation2`使用了。



**使用`Groot`实时监控行为树**

1. 打开`Groot`后选中`Monitor`。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626154730127.png)





2. 当程序跑起来后，点击左侧的`connect` 按钮连接即可显示目前正在运行的行为树。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626154923060.png)



**需要注意的是**，如果是远程查看机器的行为树状态，则要在`Server IP`中填上机器的IP地址。



**行为树log的保存与回放**

1. 保存行为树log

行为树库有以下4种log接口。其中保存为`*.fbl`格式文件的log是可以被Groot`加载并回放的。

```Bash
//4种log存储发布方式

    // Important: when the object tree goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);

    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    // This logger saves state changes on file
    FileLogger logger_file(tree, "bt_trace.fbl");//该文件可以加载到Groot中并回放

    // This logger stores the execution time of each node
    MinitraceLogger logger_minitrace(tree, "bt_trace.json");

#ifdef ZMQ_FOUND
    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree); //实时发送给Groot显示
#endif
```



2. 回放行为树log

打开`Groot`，选中`Log Replay`。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626160050539.png)



点击左侧的`Load Log`加载`fbl`格式文件即可看到log。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626160246423.png)







## 使用BT的注意事项

1. 当自定义一个叶子节点时，需要继承至合适的基类。在`Navigation2`中，对`BT`的叶子节点类型进行了封装。

同步节点封装了服务客户端，这样可以通过服务请求让相应节点做动作。

```Bash
class BtServiceNode : public BT::SyncActionNode
```

异步节点封装了动作客户端，这样可以通过动作请求让对应的服务器执行业务逻辑。

```Bash
class BtActionNode : public BT::ActionNodeBase
```



2. 行为树中的数据流

行为树中的共有数据是存放在`Blackboard`中的。`Blackboard`是以键值对的形式存储数据的。各个行为树的叶子节点均可以通过`key`访问到需要的数据。

`Ports`是用于在节点间交换数据而存在的。一个行为树叶子节点可以定义输入的`Ports`和输出的`Ports`。当不同叶子节点的`Port`有相同的`key`名称时，可以认为它们是相通的。

当在行为树叶子节点中声明了这些`Ports`时，也需要同时在`xml`文件中描述这些`Ports`。



官方对数据流的解释也放在下面了。

>  For the time being, it is important to know that:
>
>  - A Blackboard is a key/value storage shared by all the Nodes of a Tree.
>  - Ports are a mechanism that Nodes can use to exchange information between each other.
>  - Ports are "connected" using the same key of the blackboard.
>  - The number, name and kind of ports of a Node must be known at compilation-time (C++); connections between ports are done at deployment-time (XML).



下面说明一下`Blackboard`是如何使用的。



`action server`中设定`blackboard`的值

```c++
blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal->pose);

// Put items on the blackboard
blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
```



叶子节点中通过设置`ports`来设置`blackboard`中的值

```c++
static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>("planner_id", ""),
      });
  }
```

叶子节点需要使用某一个`port`时需要先在`providedPorts`这里声明，相当于在`list`里先加入。

**注意**：`portslist`中没有的`key`是不能用`getInput`或`etOutput`来操作的。



获取`port`的值

```c++
getInput("goal", goal_.goal);
getInput("planner_id", goal_.planner_id);
if (getInput("start", goal_.start)) {
    goal_.use_start = true;
}
```



设置`port`的值

```bash
setOutput("path", result_.result->path);
```



在xml文件中声明`ports`

文件路径是`nav2_behavior_tree/nav2_tree_nodes.xml`。

```xml
    <Action ID="ComputePathToPose">
      <input_port name="goal">Destination to plan to</input_port>
      <input_port name="start">Start pose of the path if overriding current robot pose</input_port>
      <output_port name="path">Path created by ComputePathToPose node</output_port>
      <input_port name="planner_id"/>
    </Action>
```



3. 每次`tickRoot()`函数执行都会遍历整个树，对于`asynActionNode`，因为其本身有循环（在单独线程里），所以循环没有结束时会返回`RUNNING`状态。不同的控制流节点对`RUNNING`的处理不一样。这一点可以查看官方文档中对控制流节点的说明。



4. `Navigation2`的`libbehaviortree_cpp_v3`依赖库默认安装在`/opt/ros/galactic/lib/libbehaviortree_cpp_v3.so` 。如果想用最新的行为树库，可以自己拉最新的代码，编译好后进行替换。需要注意新版本的接口变化哈！以免不兼容。

 

5. `BT` 并不执行特定的功能代码，它只是把各个业务逻辑代码组织起来。各个业务逻辑代码块像乐高积木一样可以按不同的想法组织起来实现不同的功能。通过不同的组织方式和少量代码的修改可以实现截然不同的功能。



### 在ROS2中使用Behavior Tree

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220626162145432.png)



在`nav2_behavior_tree`中维护了很多的插件。这些插件分成了4个类别：`action`，`condition`，`control`和`decorator`。

**action**

动作节点通常实现服务客户端和动作客户端，也可以是一些简单的执行程序。他们通过向`Planner server`，`Controller server`，`Recovery server`发送请求来启动相应的功能程序。`action`通常作为行为树中的叶子节点，负责具体行为和功能的实现。但这些具体的功能代码并没有在叶子节点中而是在对应的服务端。

**condition**

这是条件控制节点。比如判断电池电量，某一开关信号等等。

**control**

这是行为树中的控制流。类似`c++`语言中的`if else`，`switch`等等。它负责构建行为树的逻辑结构。`sequeence`，`fallback`等等就属于这个范畴。

**decorator**

`decorator`是节点装饰器。它只能有一个子节点。负责将子节点的结果进行修饰。比如将子节点的结果进行反向，约束子节点的执行次数等等。



当我们实现了足够多并且功能齐全的服务端程序后，就可以编写对应的行为树插件。通过这些插件我们将各个功能或者程序执行块进行排列组合，形成完整的功能。而逻辑上的修改可能只需要修改行为树的描述文件而并不需要改动源码。基于行为树的产品构建，或许可以让更多的人参与进产品开发而不仅仅是研发人员。



`BehaviorTreeEngine `会将参数文件中设定的插件加载好。而`nav2_bt_navigator`中维护了导航系统的行为树描述文件。

下面是`turtlebot3`的配置文件片段可作为参考。文件在`turtlebot3/turtlebot3_navigation2/param`目录中。

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
```



## 机器人开发使用BT的的优势

这里顺便总结一下使用`BT`的优势吧！

1. 调试方便，机器行为的变化都可以追溯。机器行为的变化可记录回放。行为变化也可实时监控。
2. 代码复用率高。不同的功能只需少量代码的修改和机器行为的重新组织。







## BehaviorTree相关材料

[Behavior trees for AI: How they work](https://www.gamedeveloper.com/programming/behavior-trees-for-ai-how-they-work)

[navigation2](https://github.com/ros-planning/navigation2)  

- navigation2中的[nav2_behavior_tree](https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree)模块是对[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)库的封装
- [nav2_bt_navigator](https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator)模块用于加载行为树文件并启动

[Groot 是一个可视化的行为树编辑器和调试器](https://github.com/BehaviorTree/Groot)



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
