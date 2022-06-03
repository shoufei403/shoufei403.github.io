---
title: 下载Navigation2源码来学习 #文章页面上的显示名称，一般是中文
date: 2022-05-26 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2,Navigation2] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: 下载Navigation2源码来学习
---

下面的操作是基于`galactic`

**代码下载**

```bash
#安装git和编译工具
sudo apt-get install git python3-vcstool build-essential python3-colcon-common-extensions
mkdir -p turtlebot3_ws/src
cd turtlebot3_ws/

#下载turtlebot3代码
git clone https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3 -b galactic-devel
git clone https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs -b galactic-devel
git clone https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations -b galactic-devel

#下载navigation2
git clone https://ghproxy.com/https://github.com/ros-planning/navigation2.git src/navigation2 -b galactic
git clone https://ghproxy.com/https://github.com/ros-planning/navigation2_tutorials.git src/navigation2_tutorials -b master

#下载teb_local_planner
git clone https://ghproxy.com/https://github.com/rst-tu-dortmund/costmap_converter.git src/costmap_converter -b ros2
git clone https://ghproxy.com/https://github.com/rst-tu-dortmund/teb_local_planner.git src/teb_local_planner -b ros2-master
```

注意：链接中的https://ghproxy.com/ 为使用代理下载github代码。

可选使用vcs工具下载源码：

**安装vcs**

```bash
sudo apt-get install python3-vcstool
```

在turtlebot3_ws目录下新建tmp.repos 。复制下面的内容到该文件并保存。

```YAML
repositories:
  costmap_converter:
    type: git
    url: https://ghproxy.com/https://github.com/rst-tu-dortmund/costmap_converter.git
    version: ros2
  navigation2:
    type: git
    url: https://ghproxy.com/https://github.com/ros-planning/navigation2.git
    version: galactic
  navigation2_tutorials:
    type: git
    url: https://ghproxy.com/https://github.com/ros-planning/navigation2_tutorials.git
    version: master
  teb_local_planner:
    type: git
    url: https://ghproxy.com/https://github.com/rst-tu-dortmund/teb_local_planner.git
    version: ros2-master
  turtlebot3:
    type: git
    url: https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3.git
    version: galactic-devel
  turtlebot3_msgs:
    type: git
    url: https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    version: galactic-devel
  turtlebot3_simulations:
    type: git
    url: https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    version: galactic-devel
```

turtlebot3_ws目录下运行

```Bash
vcs import src < tmp.repos
```



**解决依赖**

注意：`rosdepc`是通过小鱼的[一键安装工具](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97?lang=zh-CN)安装的。

```Bash
rosdepc update

rosdep install --from-paths src --ignore-src --rosdistro galactic -yr
```



**编译**

在turtlebot3_ws目录下source编译好的程序

```Bash
colcon build --symlink-install
```

```Bash
source install/setup.bash
```





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)



