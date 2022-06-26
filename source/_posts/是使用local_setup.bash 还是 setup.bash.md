---
title: 是使用local_setup.bash 还是 setup.bash
categories: ROS2
tags:
  - ROS2
abbrlink: 828ab83e
date: 2022-06-26 20:27:16
---


编译完`ros2`程序后，我们会发现`install`目录下有两个脚本`local_setup.bash` 和 `setup.bash`。执行程序前，通常需要`source`一下`install`目录下的脚本，以便环境变量准备就绪。这样`ros2 run`和`ros2 launch`就能找到对应的执行文件和依赖。



下面的内容是从`setup.bash`中截取出来的一段。可以发现它会先`source `一下`/opt/ros/galactic`目录再`source `一下`/home/ubuntu/turtlebot3_ws/install`目录，最后再`source`一下当前目录。这样就会把外部的多个工作空间囊括进来。如果多个工作空间中有相同名字的功能包可能就会互相冲突。
```bash
# source chained prefixes
# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced script
COLCON_CURRENT_PREFIX="/opt/ros/galactic"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"
# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced script
COLCON_CURRENT_PREFIX="/home/ubuntu/turtlebot3_ws/install"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"

# source this prefix
# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced script
COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"

unset COLCON_CURRENT_PREFIX
unset _colcon_prefix_chain_bash_source_script
```

<!--more-->


而`local_setup.bash`只会`source`脚本所在目录。这样只会查找当前`install`目录下的执行文件和依赖。

所以如果系统里有多个`ROS`工作空间，谨慎一点的方法是：

1、先`source`一下`/opt/ros/galactic/local_setup.bash`

2、在`source`一下某个用户的工作空间。



如果系统里就一个自己建的用户工作空间。那就直接`source`用户工作空间中`install`目录下的`setup.bash`。一切都搞定了。

它会帮你先`source`一下`/opt/ros/galactic/local_setup.bash`，这样就有系统的安装的`ros`执行程序和依赖了。然后再帮你`source`本用户空间的`install/local_setup.bash`。





参考：

[https://colcon.readthedocs.io/en/released/developer/environment.html#workspace-level](https://colcon.readthedocs.io/en/released/developer/environment.html#workspace-level)

[https://answers.ros.org/question/292566/what-is-the-difference-between-local_setupbash-and-setupbash/](https://answers.ros.org/question/292566/what-is-the-difference-between-local_setupbash-and-setupbash/)





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)