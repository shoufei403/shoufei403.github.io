---
title: LCM库的简单使用
categories: 实用工具
tags:
  - LCM
abbrlink: '10696932'
date: 2022-08-29 00:15:16
---

机器人软件系统中使用的通讯框架多种多样。比如，百度`Apollo`使用的`Cyber RT`，ROS1中的`TCPROS/UDPROS`通信机制，`ROS2`中使用的`DDS`等等。



下面介绍一种轻量，易用的通讯框架`LCM（Lightweight Communications and Marshalling`）。



`LCM`是一套用于消息传递和数据编码的库和工具，目标是构建高带宽和低延的实时通讯系统。它提供了一个发布/订阅消息传递模型和自动编码/解码代码的生成器。

`LCM`为多种编程语言（C/C++，C#，Java，Lua，MATLAB，Python）的应用提供接口。



`LCM`具有如下特性：

- 低延迟的进程间通信
- 使用UDP组播的高效广播机制
- 类型安全的消息编排
- 用户友好的记录和回放工具
- 没有集中的 "数据库 "或 "枢纽"--节点间直接通讯
- 没有守护进程
- 极少的依赖

<!--more-->

参考：

[https://lcm-proj.github.io/](https://lcm-proj.github.io/)



## 安装LCM

安装编译依赖

```bash
sudo apt-get install build-essential autoconf automake autopoint libglib2.0-dev libtool openjdk-8-jdk python-dev
```

编译安装

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

增加动态库链接地址

```bash
export LCM_INSTALL_DIR=/usr/local/lib
echo $LCM_INSTALL_DIR > /etc/ld.so.conf.d/lcm.conf
sudo ldconfig
```



## 定义通讯数据结构

`LCM`库通过编写`lcm`文件来定义通讯数据结构。下面是一个针对`C++`的示例。



新建`example_t.lcm` 文件，并拷贝下面的内容到文件中。

```c++
package exlcm;
struct example_t
{
    int64_t  timestamp;
    double   position[3];
    double   orientation[4]; 
    int32_t  num_ranges;
    int16_t  ranges[num_ranges];
    string   name;
    boolean  enabled;
}
```



执行下面的命令生成头文件

```bash
lcm-gen -x example_t.lcm
```



同一个`lcm`文件里可以写多个数据结构。如下所示：

```c++
package exlcm;
struct example_t
{
    int64_t  timestamp;
    double   position[3];
    double   orientation[4]; 
    int32_t  num_ranges;
    int16_t  ranges[num_ranges];
    string   name;
    boolean  enabled;
}

struct nihao_t
{
    int64_t  timest2amp;
    string   name1;
    boolean  enabl1ed;
}
```

执行`lcm-gen`后会为每一个结构体生成一个头文件。



示例来源于：

[https://lcm-proj.github.io/tut_lcmgen.html](https://lcm-proj.github.io/tut_lcmgen.html)



## 简单的使用示例



**发送代码示例**

```c++
// file: send_message.cpp
//
// LCM example program.
//
// compile with:
//  $ g++ -o send_message send_message.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ g++ -o send_message send_message.cpp `pkg-config --cflags --libs lcm`

#include <lcm/lcm-cpp.hpp>

#include "exlcm/example_t.hpp"

int main(int argc, char **argv)
{
    lcm::LCM lcm;
    if (!lcm.good())
        return 1;

    exlcm::example_t my_data;
    my_data.timestamp = 0;

    my_data.position[0] = 1;
    my_data.position[1] = 2;
    my_data.position[2] = 3;

    my_data.orientation[0] = 1;
    my_data.orientation[1] = 0;
    my_data.orientation[2] = 0;
    my_data.orientation[3] = 0;

    my_data.num_ranges = 15;
    my_data.ranges.resize(my_data.num_ranges);
    for (int i = 0; i < my_data.num_ranges; i++)
        my_data.ranges[i] = i;

    my_data.name = "example string";
    my_data.enabled = true;

    lcm.publish("EXAMPLE", &my_data);

    return 0;
}
```



**接受代码示例**

```c++
// file: listener.cpp
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener listener.cpp `pkg-config --cflags --libs lcm`

#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"

class Handler {
  public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const exlcm::example_t *msg)
    {
        int i;
        printf("Received message on channel \\"%s\\":\\n", chan.c_str());
        printf("  timestamp   = %lld\\n", (long long) msg->timestamp);
        printf("  position    = (%f, %f, %f)\\n", msg->position[0], msg->position[1],
               msg->position[2]);
        printf("  orientation = (%f, %f, %f, %f)\\n", msg->orientation[0], msg->orientation[1],
               msg->orientation[2], msg->orientation[3]);
        printf("  ranges:");
        for (i = 0; i < msg->num_ranges; i++)
            printf(" %d", msg->ranges[i]);
        printf("\\n");
        printf("  name        = '%s'\\n", msg->name.c_str());
        printf("  enabled     = %d\\n", msg->enabled);
    }
};

int main(int argc, char **argv)
{
    lcm::LCM lcm;

    if (!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("EXAMPLE", &Handler::handleMessage, &handlerObject);

    while (0 == lcm.handle()) {
        // Do nothing
    }

    return 0;
}
```



**`CMakeList.txt `的写法**

下面的写法是通用的，也适用于`ROS1`

```cmake
find_package(lcm REQUIRED)
include(${LCM_USE_FILE})
target_link_libraries(${PROJECT_NAME}
  lcm
)
```



下面的写法适用于`ROS2`

```cmake
set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_msgs"
  "OpenCV"
  "PCL"
  # "lcm"   #不要在ament_target_dependencies中加入lcm，不起作用的
  "cv_bridge"
  "image_transport"
)

ament_target_dependencies(${EXEC_NAME} ${dependencies} )  

# 不加的话会报undefined reference to `lcm_destroy' 
target_link_libraries(${EXEC_NAME} lcm)     # 使用target_link_libraries添加lcm依赖
```





## 处理回调

`lcm`用`handle()`函数来处理消息回调（接受到消息后就执行回调函数）。



`lcm.handle() `是阻塞的。只能放到单独的线程里执行。



`lcm.handleTimeout(10) `可以超时后返回，然后执行后面的代码。设置的时间的单位是毫秒。



## 进程间通讯

当需要通信的**进程分别在两台机器**里时，需要设置如下环境变量（两台电脑命令行窗口都运行这条）。

```bash
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1
```



如果需要通信的**两个进程在同一台主机**上则不需要运行上面的命令。



如果需要在**两个`docker`间实现通信**，需要在同一网络下启动这两个容器。

```bash
#以相同的网络启动容器
docker run -it --name test_docker0 --network nat shoufei/kinetic:latest /bin/bash

docker run -it --name test_docker1 --network nat shoufei/kinetic:latest /bin/bash

```

注意上面的两个`docker`启动命令均添加了`--network nat`参数。其中`nat`是用下面的命令建立的网络接口。

```bash
docker network create nat
```



**注意**，`docker`中也需要执行上面设置环境变量的命令。



## 注意事项

1. 当数据结构定义的是数组时，生成的头文件中会用`vector`来定义这个数据。进行赋值时需要先 resize vector 的长度。不然会填充数据时会报错。

```c++
void TestNode::test_tof_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  testlcm::tof_points_t data;
  
  size_t size = msg->height * msg->width;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  data.timestamp = msg->header.stamp.nanosec * 0.001;
  data.num_points = size;
  data.xs.resize(data.num_points);//当数据结构定义的是数组时，生成的头文件中会用vector来定义这个数据。进行赋值时需要先resize vector的长度。不然会填充数据时会报错。
  data.ys.resize(data.num_points);
  data.zs.resize(data.num_points);
  for(uint32_t i = 0; i < size; ++i)
  {
      data.xs[i] = temp_cloud->points[i].x;
      data.ys[i] = temp_cloud->points[i].y;
      data.zs[i] = temp_cloud->points[i].z;
  }
  RCLCPP_INFO(this->get_logger(), "got test tof points");
  lcm.publish(REAR_TOF_POINTS_MSG, &data);
}
```





参考：

[https://lcm-proj.github.io/index.html](https://lcm-proj.github.io/index.html)

[http://lcm-proj.github.io/multicast_setup.html](http://lcm-proj.github.io/multicast_setup.html)





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)