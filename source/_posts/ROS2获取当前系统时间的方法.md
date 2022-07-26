---
title: ROS2获取当前系统时间的方法
categories: ROS2
tags:
  - ROS2
abbrlink: 473a8d8c
date: 2022-07-24 08:56:08
---



## C++ 标准库中的三种时钟



**std::chrono::system_clock**



```cpp

  using namespace std::chrono_literals;
  const auto start = std::chrono::system_clock::now();


#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <chrono>
 
volatile int sink;
int main()
{
    std::cout << std::fixed << std::setprecision(9) << std::left;
    for (auto size = 1ull; size < 1000'000'000ull; size *= 100) {
        // record start time
        auto start = std::chrono::system_clock::now();
        // do some work
        std::vector<int> v(size, 42);
        sink = std::accumulate(v.begin(), v.end(), 0u); // make sure it's a side effect
        // record end time
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end - start;
        std::cout << "Time to fill and iterate a vector of " << std::setw(9)
                  << size << " ints : " << diff.count() << " s\n";
    }
}

/* 输出
Time to fill and iterate a vector of 1         ints : 0.000006568 s
Time to fill and iterate a vector of 100       ints : 0.000002854 s
Time to fill and iterate a vector of 10000     ints : 0.000116290 s
Time to fill and iterate a vector of 1000000   ints : 0.011742752 s
Time to fill and iterate a vector of 100000000 ints : 0.505534949 s
*/

//https://en.cppreference.com/w/cpp/chrono/system_clock/now
```

`system_clock`是系统范围的时钟。它是可修改的。比如同步网络时间。所以系统的时间差可能不准。

<!--more-->

**std::chrono::steady_clock**



```cpp

#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
 
int main()
{
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now - std::chrono::hours(24));
    std::cout << "24 hours ago, the time was "
              << std::put_time(std::localtime(&now_c), "%F %T") << '\n';
 
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::cout << "Hello World\n";
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Printing took "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
              << "us.\n";
}


/*输出
24 hours ago, the time was 2011-10-25 12:00:08
Hello World
Printing took 84us.
*/

https://www.apiref.com/cpp-zh/cpp/chrono/time_point.html
```

`steady_clock`是单调时钟。此时钟的时间点无法减少，像物理秒表一样。通常精度能达到纳秒级别，适合用来计算**程序执行时间**。



**std::chrono::high_resolution_clock**



```cpp
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

int main ()
{
  using namespace std::chrono;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::cout << "printing out 1000 stars...\n";
  for (int i=0; i<1000; ++i) std::cout << "*";
  std::cout << std::endl;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;

  return 0;
}

/* 输出
printing out 1000 stars...
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
****************************************
It took me 0.091001 seconds.
*/
//参考：https://cplusplus.com/reference/chrono/high_resolution_clock/now/
```

`high_resolution_clock`在不同标准库之间有不同的实现。



通常它只是 [std::chrono::steady_clock](https://www.apiref.com/cpp-zh/cpp/chrono/steady_clock.html) 或 [std::chrono::system_clock](https://www.apiref.com/cpp-zh/cpp/chrono/system_clock.html) 的别名，但实际是哪个取决于库或配置。例如对于 `gcc`的` libstdc++` 它是 `system_clock` ，对于 `MSVC` 它是 `steady_clock` ，而对于` clang `的` libc++` 它取决于配置。



所以推荐直接使用对应的时钟而不是`high_resolution_clock`。



## ROS2中的时间戳



`ROS2`中定义了三种时钟。默认是使用`RCL_SYSTEM_TIME`。它和`C++`中的`std::chrono::system_clock`是一样的，即系统时间。

```c++
typedef enum rcl_clock_type_t
{
  /// Clock uninitialized
  RCL_CLOCK_UNINITIALIZED = 0,
  /// Use ROS time
  RCL_ROS_TIME,
  /// Use system time
  RCL_SYSTEM_TIME,
  /// Use a steady clock time
  RCL_STEADY_TIME
} rcl_clock_type_t;
```



`ROS2`中存在两种时间戳。一种是实际的物理系统时间，另一种是仿真时间。仿真时间通常是`Gazebo`发出的`/clock`话题。

`/clock`话题

```bash
clock:
  sec: 65
  nanosec: 212000000
---
clock:
  sec: 65
  nanosec: 298000000
---
clock:
  sec: 65
  nanosec: 388000000

```



当需要在仿真环境中测试程序时，要将`use_sim_time`置为`True`。不然会出现下面的报错信息。表示时间不匹配。`tf`关系不能正常找到。

报错信息：

```bash
[controller_server]: Extrapolation Error: Lookup would require extrapolation into the future.  Requested time 1646190408.221065 but the latest data is at time 19.413000, when looking up transform from frame [map] to frame [odom]
```



置位`use_sim_time`可以写在`launch`文件中。如下所示：

```python
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),


        Node(
            package='time_api_test',
            executable='time_api_test',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[],
            output='screen'),
    ])
```



**在`ROS2`中获取当前时间戳**

下面的代码片段罗列了获取系统时间的几种方法：

```c++
    auto t = rclcpp::Clock().now();
    RCLCPP_INFO(this->get_logger(), "[rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());

    auto t1 = std::chrono::system_clock::now(); 
    time_t tt = std::chrono::system_clock::to_time_t ( t1 );
    RCLCPP_INFO(this->get_logger(), "[std::chrono::system_clock::now()] sec:%ld", tt);

    std::chrono::steady_clock::time_point td = std::chrono::steady_clock::now(); 
    std::chrono::steady_clock::duration dtn = td.time_since_epoch();
    double secs = dtn.count() * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
    RCLCPP_INFO(this->get_logger(), "[std::chrono::steady_clock::now()] sec:%lf", secs);
    
    auto t2 = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "[get_clock()->now()] sec:%lf nano:%ld", t2.seconds(), t2.nanoseconds());

    auto t3 = this->now();
    RCLCPP_INFO(this->get_logger(), "[this->now()] sec:%lf nano:%ld", t3.seconds(), t3.nanoseconds());
```



**注意**，完整的示例代码可在公众号《首飞》中回复“time” 获取到。



当`use_sim_time`为`false`时，运行上面测试代码的结果为：

```bash
[time_api_test-1] [INFO 1658498046.378262276] [time_api_test]: [rclcpp::Clock().now()] sec:1658498046.378260 nano:1658498046378260389 
[time_api_test-1] [INFO 1658498046.378315552] [time_api_test]: [std::chrono::system_clock::now()] sec:1658498046 
[time_api_test-1] [INFO 1658498046.378325479] [time_api_test]: [std::chrono::steady_clock::now()] sec:5906.000000 
[time_api_test-1] [INFO 1658498046.378333539] [time_api_test]: [get_clock()->now()] sec:1658498046.378333 nano:1658498046378333309 
[time_api_test-1] [INFO 1658498046.378341987] [time_api_test]: [this->now()] sec:1658498046.378342 nano:1658498046378341769 
```

**可以发现**，`rclcpp::Clock().now()`，`get_clock()->now()`和`this->now()`获取到的时间与`std::chrono::system_clock::now()`是一致的。

这里需要**注意的一点是**，`rclcpp::Clock().now()`，`get_clock()->now()`和`this->now()`获取到的时间戳均包含`seconds()`和`nanoseconds()`方法。`seconds()`和`nanoseconds()`方法都返回了当前的时间，是等价的，只是单位不一样。一个是以秒为单位，一个是纳秒为单位。

```c++
auto t = this->get_clock()->now()

builtin_interfaces::msg::Time tt = t;
    
RCLCPP_INFO(this->get_logger(), "sec: %lf nano: %lf  tt_sec: %ld tt_nano: %ld", t.seconds(), t.nanoseconds(), tt.sec, tt.nanosec);
```



但是`ROS2`中的时间类型`builtin_interfaces::msg::Time`是需要把秒和纳秒组合起来才能表示当前时间的。

```c++
double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
```

`now_sec`是以秒为单位的时间。



当`use_sim_time`为`true`时，运行上面测试代码的结果为：

注意，运行测试代码前。我运行了`turtlebot3`的`gazebo`仿真环境。以便系统中有`/clock`话题来提供仿真时间。

```bash
[time_api_test-1] [INFO 1658498456.593959180] [time_api_test]: [rclcpp::Clock().now()] sec:1658498456.593951 nano:1658498456593951346 
[time_api_test-1] [INFO 1658498456.594114487] [time_api_test]: [std::chrono::system_clock::now()] sec:1658498456 
[time_api_test-1] [INFO 1658498456.594132577] [time_api_test]: [std::chrono::steady_clock::now()] sec:6316.000000 
[time_api_test-1] [INFO 1658498456.594142330] [time_api_test]: [get_clock()->now()] sec:300.636000 nano:300636000000 
[time_api_test-1] [INFO 1658498456.594149351] [time_api_test]: [this->now()] sec:300.636000 nano:300636000000 
```



**可以看到**，`rclcpp::Clock().now()`还是与`std::chrono::system_clock::now()`保持一致。说明`rclcpp::Clock().now()`不能返回仿真时间。

`get_clock()->now()`和`this->now()`是等效的。他们均返回了仿真时间。



经过测试对比，**可得出结论**，`rclcpp::Clock().now()`无法正确获取仿真时间。所以代码中要获取时间戳时，可调用`get_clock()->now()`和`this->now()`接口。这样可以保证在标志位`use_sim_time`变化时，代码各处使用的时间戳是一致的。



**计算程序运行时间的方法**



```c++
rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

auto start_time = steady_clock_.now();
//do something ...
auto cycle_duration = steady_clock_.now() - start_time;

RCLCPP_INFO(get_logger(), "Cost %.4f s", cycle_duration.seconds());
```

使用`RCL_STEADY_TIME`时钟去计算程序运行时间是比较准确的。



---

**觉得有用就点赞吧！**



我是首飞，一个帮大家**填坑**的机器人开发攻城狮。



另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)



