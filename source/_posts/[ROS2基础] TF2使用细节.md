---
title: ROS2基础之TF2使用细节
categories: ROS2
tags:
  - ROS2
abbrlink: 71136b95
date: 2022-06-05 21:15:16
---

## 运行一个示例

#### 安装依赖

```bash
sudo apt-get install ros-galactic-turtle-tf2-py ros-galactic-tf2-tools ros-galactic-tf-transformations
```

```bash
pip3 install transforms3d
```

#### 运行示例

在不同的命令窗口中运行下面的命令

启动小乌龟窗口

```bash
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
```

启动键盘控制节点

```bash
ros2 run turtlesim turtle_teleop_key
```

观察坐标转换的结果

```bash
ros2 run tf2_ros tf2_echo turtle2 turtle1
```

#### 示例分析

本示例中启动了两只小乌龟`Turtle1`和`Turtle2`。`TF`发布器会将`Turtle1`相对于`world`坐标系的位置关系和`Turtle2`相对于`world`坐标系的位置关系发布出来。为了实现`Turtle2`跟随`Turtle1`的效果，程序中获取了`Turtle1`相对于`Turtle2`的位置关系并且将其折算成速度控制量。
<!--more-->
```bash
        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);//这里得到的是from_frame_rel在to_frame_rel坐标系下的相对位置
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
```

代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_cpp/src/turtle_tf2_listener.cpp`。

上面的示例代码可通过下面的方式获取：

```bash
git clone https://gitee.com/shoufei/ros2_galactic_turorials.git
```

或者

```bash
git clone git@github.com:shoufei403/ros2_galactic_tutorials.git
```



## TF2树



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/turtlesim_frames.png)

`tf2`树表征了各个link的位置关系。一个`link`有且只有一个`parent link`。所以`tf2`树是不会形成**闭环**的。

```bash
ros2 run tf2_tools view_frames
```

用这个命令可以保存当前系统中`tf2`树的关系图（以`pdf`文件的形式保存在**运行命令的目录**下）。



## 静态TF发布器和动态TF发布器

#### 静态TF发布器

##### 编写代码发布TF（C++）

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_cpp/src/static_turtle_tf2_broadcaster.cpp`

```c++
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

using std::placeholders::_1;

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher(char * transformation[])
  : Node("static_turtle_tf2_broadcaster")
  {
    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms(transformation);
  }

private:
  void make_transforms(char * transformation[])
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(
      atof(transformation[5]),
      atof(transformation[6]),
      atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_publisher_->sendTransform(t);
  }
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc != 8) {
    RCLCPP_INFO(
      logger, "Invalid number of parameters\nusage: "
      "ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
      "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // As the parent frame of the transform is `world`, it is
  // necessary to check that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
    return 1;
  }

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  rclcpp::shutdown();
  return 0;
}

```

##### 编写代码代码发布TF（Python）

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py`

```python
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf_transformations

from turtlesim.msg import Pose


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter
        self.declare_parameter('turtlename', 'turtle')
        self.turtlename = self.get_parameter(
            'turtlename').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)


    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

```

##### 手动以命令行形式发布TF

**角度用欧拉角表示**

`x/y/z `偏移的单位是米，`roll/pitch/yaw`偏移的单位是`rad`。

> Publish a static coordinate transform to tf2 using an x/y/z offset in meters and roll/pitch/yaw in radians. In our case, roll/pitch/yaw refers to rotation about the x/y/z-axis, respectively.



```bash
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
```

**角度用四元数表示**

> Publish a static coordinate transform to tf2 using an x/y/z offset in meters and quaternion.

```bash
ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
```

##### 在launch文件中发布TF

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '1', '0', '0', '0', 'world', 'mystaticturtle']
      ),
   ])
```



#### 动态TF发布器

动态的tf发布一般是编写代码来实现的。

##### C++实现动态TF发布

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_cpp/src/turtle_tf2_broadcaster.cpp`

```C++
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    this->declare_parameter<std::string>("turtlename", "turtle");
    this->get_parameter("turtlename", turtlename_);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_turtle_pose, this, _1));
  }

private:
  void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_.c_str();

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}

```



##### Python实现动态TF发布

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py`

```python
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf_transformations

from turtlesim.msg import Pose


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter
        self.declare_parameter('turtlename', 'turtle')
        self.turtlename = self.get_parameter(
            'turtlename').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)


    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

```



## 监听TF数据

**实际上**，就是是获取两个坐标系之间的相对位置。

C++版本

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_cpp/src/turtle_tf2_listener.cpp`

```C++
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <turtlesim/srv/spawn.hpp>

#include <chrono>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    // Declare and acquire `target_frame` parameter
    this->declare_parameter<std::string>("target_frame", "turtle1");
    this->get_parameter("target_frame", target_frame_);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ =
      this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";

    if (turtle_spawning_service_ready_) {
      if (turtle_spawned_) {
        geometry_msgs::msg::TransformStamped transformStamped;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);//这里得到的是from_frame_rel在to_frame_rel坐标系下的位置
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        geometry_msgs::msg::Twist msg;

        static const double scaleRotationRate = 1.0;
        msg.angular.z = scaleRotationRate * atan2(
          transformStamped.transform.translation.y,
          transformStamped.transform.translation.x);

        static const double scaleForwardSpeed = 0.5;
        msg.linear.x = scaleForwardSpeed * sqrt(
          pow(transformStamped.transform.translation.x, 2) +
          pow(transformStamped.transform.translation.y, 2));

        publisher_->publish(msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully spawned");
        turtle_spawned_ = true;
      }
    } else {
      // Check if the service is ready
      if (spawner_->service_is_ready()) {
        // Initialize request with turtle name and coordinates
        // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 4.0;
        request->y = 2.0;
        request->theta = 0.0;
        request->name = "turtle2";

        // Call request
        using ServiceResponseFuture =
          rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (strcmp(result->name.c_str(), "turtle2") == 0) {
              turtle_spawning_service_ready_ = true;
            } else {
              RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
            }
          };
        auto result = spawner_->async_send_request(request, response_received_callback);
      } else {
        RCLCPP_INFO(this->get_logger(), "Service is not ready");
      }
    }
  }
  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  bool turtle_spawned_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}

```



增加超时时间的写法

```c++
        try {
          rclcpp::Time now = this->get_clock()->now();
          transformStamped = tf_buffer_->lookupTransform(
            toFrameRel,
            fromFrameRel,
            now,
            50ms);//获取当前时间的tf关系并且允许50ms的超时时间
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
```

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_cpp/src/turtle_tf2_listener_timeout.cpp`



可运行下面的命令查看效果

```bash
ros2 launch turtle_tf2_cpp turtle_tf2_demo_timeout.launch.py
```

```bash
ros2 run turtlesim turtle_teleop_key
```





获取历史时间点的`tf`关系

```c++
        try {
          rclcpp::Time now = this->get_clock()->now();
          rclcpp::Time when = now - rclcpp::Duration(5, 0);
          transformStamped = tf_buffer_->lookupTransform(
              toFrameRel,
              now,
              fromFrameRel,
              when,
              "world",
              50ms);
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
```

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_cpp/src/turtle_tf2_listener_time_travel.cpp`

运行命令

```bash
ros2 launch turtle_tf2_cpp turtle_tf2_demo_time_travel.launch.py
```

```bash
ros2 run turtlesim turtle_teleop_key
```





python版本

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py`

```python
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'turtle1')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)  #这里得到的是from_frame_rel在to_frame_rel坐标系下的位置
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    trans.transform.translation.y,
                    trans.transform.translation.x)

                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    trans.transform.translation.x ** 2 +
                    trans.transform.translation.y ** 2)

                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

```



增加超时时间的写法

```python
trans = self._tf_buffer.lookup_transform(
   to_frame_rel,
   from_frame_rel,
   now,
   timeout=rclpy.time.Duration(seconds=1.0))
```

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener_timeout.py`

运行命令

```bash
ros2 launch turtle_tf2_py turtle_tf2_demo_timeout.launch.py
```

```bash
ros2 run turtlesim turtle_teleop_key
```



获取历史时间点的tf关系

```python
                try:
                    when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)
                    trans = self.tf_buffer.lookup_transform_full(
                            target_frame=to_frame_rel,
                            target_time=rclpy.time.Time(),
                            source_frame=from_frame_rel,
                            source_time=when,
                            fixed_frame='world',
                            timeout=rclpy.time.Duration(seconds=0.05))
                except (LookupException, ConnectivityException, ExtrapolationException):
                    self.get_logger().info('transform not ready')
                    return
```

示例代码来自`ros2_galactic_turorials/geometry_tutorials/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener_time_travel.py`

运行命令

```bash
ros2 launch turtle_tf2_py turtle_tf2_demo_time_travel.launch.py
```

```bash
ros2 run turtlesim turtle_teleop_key
```



## TF 调试工具

打印两个`link`的相对位置关系

```bash
ros2 run tf2_ros tf2_echo turtle2 turtle1
```

保存`tf`关系框图

```bash
ros2 run tf2_tools view_frames
```

监控两个`link`的转换延时

```bash
ros2 run tf2_ros tf2_monitor turtle2 turtle1
```



## 四元数与欧拉角转换

因为使用`TF`的过程中常常涉及到坐标转换。这会涉及到角度的表示方法。

通常我们会有两种表示角度的方法，一种是欧拉角，一种是四元数。欧拉角是更为直观的表示方法，但因为**死锁的问题**，在进行坐标变换计算时常常是使用四元数的。

在`ROS2`中，有两个数据类型表示四元数。它们是来自`tf2`功能包的`tf2::Quaternion`类型和来自`geometry_msgs`功能包的`geometry_msgs::msg::Quaternion`类型。我们可以通过`tf2_geometry_msgs`包来进行两个类型的互相转换。

`C++`的写法

```c++
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
...

tf2::Quaternion tf2_quat, tf2_quat_from_msg;
tf2_quat.setRPY(roll, pitch, yaw);
// Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

// Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
tf2::convert(msg_quat, tf2_quat_from_msg);
// or
tf2::fromMsg(msg_quat, tf2_quat_from_msg);
```

`Python`的写法

```pyth
from geometry_msgs.msg import Quaternion
...

# Create a list of floats, which is compatible with tf2
# Quaternion methods
quat_tf = [0.0, 1.0, 0.0, 0.0]

# Convert a list to geometry_msgs.msg.Quaternion
msg_quat = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
```



欧拉角和四元数之间的转换可以用下面的方式

**欧拉角转四元数**

`C++`版本

```c++
#include <tf2/LinearMath/Quaternion.h>

tf2::Quaternion myQuaternion;
myQuaternion.setRPY(1.5707, 0, -1.5707);  // Create this quaternion from roll/pitch/yaw (in radians)
ROS_INFO("%f  %f  %f  %f" ,myQuaternion.x(),myQuaternion.y(),myQuaternion.z(),myQuaternion.w());  // Print the quaternion components (0,0,0,1)
```

`Python`版本

```python
import tf_transformations
...

q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
print(f'The quaternion representation is x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]}.')
```



**四元数转欧拉角**

`C++`版本

在`tf2/utils.h`中定义了如下方法。

```c++
/** Return the yaw, pitch, roll of anything that can be converted to a tf2::Quaternion
 * The conventions are the usual ROS ones defined in tf2/LineMath/Matrix3x3.h
 * \param a the object to get data from (it represents a rotation/quaternion)
 * \param yaw yaw
 * \param pitch pitch
 * \param roll roll
 */
template<class A>
void getEulerYPR(const A & a, double & yaw, double & pitch, double & roll)
{
  tf2::Quaternion q = impl::toQuaternion(a);
  impl::getEulerYPR(q, yaw, pitch, roll);
}

/** Return the yaw of anything that can be converted to a tf2::Quaternion
 * The conventions are the usual ROS ones defined in tf2/LineMath/Matrix3x3.h
 * This function is a specialization of getEulerYPR and is useful for its
 * wide-spread use in navigation
 * \param a the object to get data from (it represents a rotation/quaternion)
 * \param yaw yaw
 */
template<class A>
double getYaw(const A & a)
{
  tf2::Quaternion q = impl::toQuaternion(a);
  return impl::getYaw(q);
}
```

`Python`版本

```python
import tf_transformations

rpy = tf_transformations.euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
```


更多细节请查看下面的文章：
[https://docs.ros.org/en/galactic/Tutorials/Tf2/Quaternion-Fundamentals.html](https://docs.ros.org/en/galactic/Tutorials/Tf2/Quaternion-Fundamentals.html)



## tf2_ros::MessageFilter

[https://docs.ros.org/en/galactic/Tutorials/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html](https://docs.ros.org/en/galactic/Tutorials/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html)

> To do this `turtle1` must listen to the topic where `turtle3`’s pose is being published, wait until transforms into the desired frame are ready, and then do its operations. To make this easier the `tf2_ros::MessageFilter` is very useful. The `tf2_ros::MessageFilter` will take a subscription to any ROS 2 message with a header and cache it until it is possible to transform it into the target frame.

`tf2_ros::MessageFilter` 的主要作用是，等待目标`frame`已经缓冲在`tf`树中。这样的话转换到该`frame`将是可行的。`AMCL`模块中处理激光数据就是一个好的例子。

```c++
void
AmclNode::initMessageFilters()
{
  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    rclcpp_node_.get(), scan_topic_, rmw_qos_profile_sensor_data);

  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10, rclcpp_node_, transform_tolerance_);

  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(
      &AmclNode::laserReceived,
      this, std::placeholders::_1));
}
```

当`odom_frame_id_`在`tf`树中可转换时，接受到激光数据后开始执行回调函数`laserReceived` 。在`laserReceived` 中通过`odom_frame_id_` 获取到当前的`odom` 数据。也就是说，使用`tf2_ros::MessageFilter`  达到了，等能获取`odom` 数据时再处理激光数据的效果。



下面是官方的一个示例。可以参考其写法。

```c++
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#ifdef TF2_CPP_HEADERS
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class PoseDrawer : public rclcpp::Node
{
public:
  PoseDrawer()
  : Node("turtle_tf2_pose_drawer")
  {
    // Declare and acquire `target_frame` parameter
    this->declare_parameter<std::string>("target_frame", "turtle1");
    this->get_parameter("target_frame", target_frame_);

    typedef std::chrono::duration<int> seconds_type;
    seconds_type buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    point_sub_.subscribe(this, "/turtle3/turtle_point_stamped");
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
      point_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
      this->get_node_clock_interface(), buffer_timeout);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&PoseDrawer::msgCallback, this);//等待需要的tf关系就绪，一就绪有数据就进入到回调函数中。
  }

private:
  void msgCallback(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr)
  {
    geometry_msgs::msg::PointStamped point_out;
    try {
      tf2_buffer_->transform(*point_ptr, point_out, target_frame_);//point_out的值是turtle3想对于target_frame_的坐标
      RCLCPP_INFO(
        this->get_logger(), "Point of turtle3 in frame of turtle1: x:%f y:%f z:%f\n",
        point_out.point.x,
        point_out.point.y,
        point_out.point.z);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        // Print exception which was caught
        this->get_logger(), "Failure %s\n", ex.what());
    }
  }
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseDrawer>());
  rclcpp::shutdown();
  return 0;
}
```



参考：

[https://docs.ros.org/en/galactic/Tutorials/Tf2/Tf2-Main.html](https://docs.ros.org/en/galactic/Tutorials/Tf2/Tf2-Main.html)



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)