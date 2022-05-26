[ROS2]源码安装turtlebot3用于调试（简易版）



下面的操作是基于**ROS2 galactic**

#### turtlebot3的代码下载：

**安装git，编译工具和下载源代码**

```Bash
sudo apt-get install git python3-vcstool build-essential python3-colcon-common-extensions
mkdir -p turtlebot3_ws/src
cd turtlebot3_ws/
git clone https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3 -b galactic-devel
git clone https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs -b galactic-devel
git clone https://ghproxy.com/https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations -b galactic-devel
```

注意：链接中的[https://ghproxy.com/](https://ghproxy.com/) 为使用代理下载github代码。

**可选使用vcs工具下载源码：**

安装vcs

```Apache
sudo apt-get install python3-vcstool
```

在turtlebot3_ws目录下新建tmp.repos 。复制下面的内容到该文件并保存。

```YAML
repositories:
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

```Bash
rosdep update

rosdep install --from-paths src --ignore-src --rosdistro galactic -yr
sudo apt-get install ros-galactic-gazebo-*
```

**编译**

```Bash
colcon build --symlink-install
```

在turtlebot3_ws目录下source编译好的程序

```Bash
source install/setup.bash
```