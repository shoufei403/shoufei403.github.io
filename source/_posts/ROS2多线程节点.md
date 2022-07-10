---
title: ROS2多线程节点
categories: ROS2
tags:
  - ROS2
abbrlink: 9708374f
date: 2022-07-10 21:50:16
---



下面介绍一下如何在`ROS2`节点中使用多线程。

使用多线程就涉及到回调组（`CallbackGroup`）了。



## 使用示例

创建回调组的函数如下：

```c++
  /// Create and return a callback group.
  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr
  create_callback_group(
    rclcpp::CallbackGroupType group_type,
    bool automatically_add_to_executor_with_node = true);
```

<!--more-->

可以看到，创建回调组时是可以选择使用哪种类型的回调组（`CallbackGroup`）的。回调组的类型如下：



```c++
enum class CallbackGroupType
{
  MutuallyExclusive,
  Reentrant
};
```

`MutuallyExclusive`表示此组的回调函数是互斥的，不能在同一时间被执行。`Reentrant`表示回调函数是可重入的，允许同一时刻被多次执行。通常使用的还是`MutuallyExclusive`类型。



创建回调组回调组的另外一个参数是`automatically_add_to_executor_with_node`。它的默认值是`true`。

这个参数决定了回调组绑定`node`的方式。



当`automatically_add_to_executor_with_node`为`true`时，采用在节点外部使用`add_node`的方式绑定`node`。可查看下面的示例程序。

`examples/rclcpp/executors/multithreaded_executor/multithreaded_executor.cpp`

```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * A small convenience function for converting a thread ID to a string
 **/
std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

/* For this example, we will be creating a publishing node like the one in minimal_publisher.
 * We will have a single subscriber node running 2 threads. Each thread loops at different speeds, and
 * just repeats what it sees from the publisher to the screen.
 */

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
  : Node("PublisherNode"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello World! " + std::to_string(this->count_++);

        // Extract current thread
        auto curr_thread = string_thread_id();

        // Prep display message
        RCLCPP_INFO(
          this->get_logger(), "\n<<THREAD %s>> Publishing '%s'",
          curr_thread.c_str(), message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

class DualThreadedNode : public rclcpp::Node
{
public:
  DualThreadedNode()
  : Node("DualThreadedNode")
  {
    /* These define the callback groups
     * They don't really do much on their own, but they have to exist in order to
     * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
     */
    callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_subscriber2_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_subscriber2_;

    subscription1_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      rclcpp::QoS(10),
      // std::bind is sort of C++'s way of passing a function
      // If you're used to function-passing, skip these comments
      std::bind(
        &DualThreadedNode::subscriber1_cb,  // First parameter is a reference to the function
        this,                               // What the function should be bound to
        std::placeholders::_1),             // At this point we're not positive of all the
                                            // parameters being passed
                                            // So we just put a generic placeholder
                                            // into the binder
                                            // (since we know we need ONE parameter)
      sub1_opt);                  // This is where we set the callback group.
                                  // This subscription will run with callback group subscriber1

    subscription2_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      rclcpp::QoS(10),
      std::bind(
        &DualThreadedNode::subscriber2_cb,
        this,
        std::placeholders::_1),
      sub2_opt);
  }

private:
  /**
   * Simple function for generating a timestamp
   * Used for somewhat ineffectually demonstrating that the multithreading doesn't cripple performace
   */
  std::string timing_string()
  {
    rclcpp::Time time = this->now();
    return std::to_string(time.nanoseconds());
  }

  /**
   * Every time the Publisher publishes something, all subscribers to the topic get poked
   * This function gets called when Subscriber1 is poked (due to the std::bind we used when defining it)
   */
  void subscriber1_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    auto message_received_at = timing_string();

    // Extract current thread
    RCLCPP_INFO(
      this->get_logger(), "THREAD %s => Heard '%s' at %s",
      string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
  }

  /**
   * This function gets called when Subscriber2 is poked
   * Since it's running on a separate thread than Subscriber 1, it will run at (more-or-less) the same time!
   */
  void subscriber2_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    auto message_received_at = timing_string();

    // Prep display message
    RCLCPP_INFO(
      this->get_logger(), "THREAD %s => Heard '%s' at %s",
      string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // You MUST use the MultiThreadedExecutor to use, well, multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  auto pubnode = std::make_shared<PublisherNode>();
  auto subnode = std::make_shared<DualThreadedNode>();  // This contains BOTH subscriber callbacks.
                                                        // They will still run on different threads
                                                        // One Node. Two callbacks. Two Threads
  executor.add_node(pubnode);
  executor.add_node(subnode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

```



当一个节点中有多个线程时，需要用到`rclcpp::executors::MultiThreadedExecutor`。上面示例程序中，`DualThreadedNode`是有两个线程的。这两个线程中分别运行一个订阅器的回调函数。两个线程独立运行互不干扰。



上面示例程序的完整版可使用下面的方式获取：

```bash
git clone https://github.com/shoufei403/ros2_galactic_tutorials.git
```



下载编译好后，可使用下面的命令运行测试。

```bash
ros2 run examples_rclcpp_multithreaded_executor
```



输出结果：

```bash
[PublisherNode]: 
<<THREAD 6504961969737349918>> Publishing 'Hello World! 0' 
[DualThreadedNode]: THREAD 3314359393590349369 => Heard 'Hello World! 0' at 1657449288147060106 
[DualThreadedNode]: THREAD 6504961969737349918 => Heard 'Hello World! 0' at 1657449288147082759 
[PublisherNode]: 
<<THREAD 16625943778230753959>> Publishing 'Hello World! 1' 
[DualThreadedNode]: THREAD 13431477624131009972 => Heard 'Hello World! 1' at 1657449288646978265 
[DualThreadedNode]: THREAD 16625943778230753959 => Heard 'Hello World! 1' at 1657449288647097545 
[PublisherNode]: 
<<THREAD 12195914629433846612>> Publishing 'Hello World! 2' 
[DualThreadedNode]: THREAD 17256547440473779954 => Heard 'Hello World! 2' at 1657449289146970672 
[DualThreadedNode]: THREAD 12195914629433846612 => Heard 'Hello World! 2' at 1657449289147078711 
```



可以看到回调函数是执行**在不同的线程**中的。



当`automatically_add_to_executor_with_node`为`false`时，采用在节点内部使用`add_callback_group`的方式绑定`node`。可查看下面的示例程序。



`navigation2/nav2_behavior_tree/plugins/condition/is_battery_low_condition.cpp`

```c++
#include <string>

#include "nav2_behavior_tree/plugins/condition/is_battery_low_condition.hpp"

namespace nav2_behavior_tree
{

IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
  getInput("min_battery", min_battery_);
  getInput("battery_topic", battery_topic_);
  getInput("is_voltage", is_voltage_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_battery_low_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryLowCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (is_voltage_) {
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}

```



`navigation2/nav2_behavior_tree/include/nav2_behavior_tree/plugins/condition/is_battery_low_condition.hpp`

```c++
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is low and FAILURE otherwise
 */
class IsBatteryLowCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsBatteryLowCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryLowCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery", "Minimum battery percentage/voltage"),
      BT::InputPort<std::string>(
        "battery_topic", std::string("/battery_status"), "Battery topic"),
      BT::InputPort<bool>(
        "is_voltage", false, "If true voltage will be used to check for low battery"),
    };
  }

private:
  /**
   * @brief Callback function for battery topic
   * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
   */
  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::string battery_topic_;
  double min_battery_;
  bool is_voltage_;
  bool is_battery_low_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

```



**可以看到**，这里使用的是`rclcpp::executors::SingleThreadedExecutor`（单线程执行器）。





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
