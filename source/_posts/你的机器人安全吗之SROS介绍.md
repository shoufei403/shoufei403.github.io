---
title: 你的机器人安全吗之SROS介绍
categories: ROS2
tags:
  - SROS2
abbrlink: 3cad4509
date: 2022-08-17 09:11:08
---

你的机器人安全吗？

ROS1中的通信数据基本是开放的。只要我们知道机器的IP。然后使用下面的命令在自己的笔记本电脑里声明一下机器人系统中`ROS_MASTER`的地址。

```bash
export ROS_MASTER_URI="http://192.7.8.48:11311"
```

注意，这里的ip（192.7.8.48）需要设成实际机器的ip。

这样我们就可以获取机器人系统的所有通信数据。

并且**很危险的是**，我们可以在自己的电脑中向机器人系统发送任意速度指令或者其他指令来干扰机器人的正常运行。

想想就很不安全。



`ROS2`中默认集成了`SROS2`功能包。它可以将通信数据进行加密从而保证数据安全。下面来认识一下这个功能包。

`SROS2`仓库地址：[https://github.com/ros2/sros2](https://github.com/ros2/sros2)


<!--more-->
## SROS2介绍

众所周知，`ROS2`的通信体系是建立在`DDS`上的。而`DDS`本身是有一些安全机制的，即`DDS-Security`。`SROS2`就是那个可以让我们更方便使用`DDS-Security`特性的功能包。它提供了一些方便的工具。



**通讯加密后，通讯延时会加长。**如下图所示：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220806225037730.png)

通过上图可以看到，加密传输的耗时要多于非加密传输。这也是不可避免的。毕竟加密后数据包也变大了。因为要包含加密数据。



**跨设备间也可以加密通信。**我们的笔记本电脑和机器间也是可以加密通讯的。只要把机器内生成的加密文件拷贝到笔记本电脑，然后参考下面描述的步骤进行操作即可。（主要是设置环境变量，指明加密文件的路径）





## 一个简单的示例



1. **生成一个存储加密文件的目录**

```bash
ros2 security create_keystore test_keystore
```

注意，其中`test_keystore`为目录名称



2. **不加密的情况下运行消息收发示例**

```bash
ros2 run demo_nodes_cpp talker
```



```bash
ros2 run demo_nodes_py listener
```



3. **运行命令生成加密策略文件**

**注意**，是在上面的节点运行的情况下运行下面的命令。这样能生成针对整个系统的加密策略文件。

```bash
ros2 security generate_policy test_policy.xml
```



生成的文件内容如下：

```xml
<policy version="0.2.0">
  <enclaves>
    <enclave path="/">
      <profiles>
        <profile node="listener" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>chatter</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
        <profile node="talker" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>chatter</topic>
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```



4. **根据加密策略文件生成加密文件**

```bash
ros2 security generate_artifacts -k test_keystore -p test_policy.xml -e /talk_listener
```

`-k`参数指定加密文件目录的根目录，`-p`参数指定加密策略文件，`-e`参数指定生成的`enclave`名称。需要注意的是，`talk_listener`前面必须加`/`。

或者也可以直接修改加密策略文件中的`<enclave path="/">`来设置`enclave`的名称。`e`选项的内容最好定义在`xml`文件中。

当加密策略文件变动时，就需要重新生成加密文件。

**重新生成加密文件时**，只需删掉之前生成的`talk_listener`目录，然后重新生成即可。





5. **设置环境变量**

```bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE=/home/kevin/turtlebot3_demo/test_keystore
```

上面设置环境变量的语句可以加入到`~/.bashrc`中。



`ROS_SECURITY_ENABLE`是开启加密功能的总开关。



`ROS_SECURITY_STRATEGY`有两个策略。默认的策略是**Permissive mode**。在这个策略下，如果一个节点启动时没有指定或找到相应的加密解密文件，它就会以不加密的形式启动。如果设置的策略是**Strict mode**，没有指定或找到相应的加密解密文件时，节点是不能启动的（启动时会报错）。设置`ROS_SECURITY_STRATEGY`为`Enforce`即可开启**Strict mode**模式。



`ROS_SECURITY_KEYSTORE`用于设定加密文件的根目录。

更多细节可以参考下面的网页：  

[https://design.ros2.org/articles/ros2_dds_security.html](https://design.ros2.org/articles/ros2_dds_security.html)



效果演示：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/sros2_test.gif)

左侧和右上开启了加密功能，右下命令窗口没有开启加密功能。**可以看到**，右下命令窗口是接收不到消息的。



## 加密后相关工具是否还可用



**加密后rosbag 是否可用**

记录包时的打印信息：

```bash
➜  turtlebot3_demo git:(master) ✗ ros2 bag record -a                                                                             
[INFO 1660520472.608949340] [rcl]: Found security directory: /home/kevin/turtlebot3_demo/test_keystore/enclaves (rcl_get_security_options_from_environment() at /tmp/binarydeb/ros-galactic-rcl-3.1.2/src/rcl/security.c:56)
1660520472.617346 [0]       ros2: using network interface wlp109s0 (udp/192.168.31.206) selected arbitrarily from: wlp109s0, anbox0, br-1f67ea43b642, br-fc22da249d2c, br-14287a178700, docker0
[INFO 1660520472.625432594] [rosbag2_storage]: Opened database 'rosbag2_2022_08_15-07_41_12/rosbag2_2022_08_15-07_41_12_0.db3' for READ_WRITE. (open() at /tmp/binarydeb/ros-galactic-rosbag2-storage-default-plugins-0.9.1/src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp:216)
[INFO 1660520472.625510286] [rosbag2_recorder]: Listening for topics... (record() at /tmp/binarydeb/ros-galactic-rosbag2-transport-0.9.1/src/rosbag2_transport/recorder.cpp:97)
[INFO 1660520472.626160066] [rosbag2_recorder]: Subscribed to topic '/rosout' (subscribe_topic() at /tmp/binarydeb/ros-galactic-rosbag2-transport-0.9.1/src/rosbag2_transport/recorder.cpp:214)
[INFO 1660520472.626445742] [rosbag2_recorder]: Subscribed to topic '/parameter_events' (subscribe_topic() at /tmp/binarydeb/ros-galactic-rosbag2-transport-0.9.1/src/rosbag2_transport/recorder.cpp:214)
[INFO 1660520472.829706666] [rosbag2_recorder]: Subscribed to topic '/chatter' (subscribe_topic() at /tmp/binarydeb/ros-galactic-rosbag2-transport-0.9.1/src/rosbag2_transport/recorder.cpp:214)
^C[INFO 1660520503.667573993] [rclcpp]: signal_handler(signal_value=2) (signal_handler() at /tmp/binarydeb/ros-galactic-rclcpp-9.2.0/src/rclcpp/signal_handler.cpp:202)
[INFO 1660520503.747822482] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while (close() at /tmp/binarydeb/ros-galactic-rosbag2-cpp-0.9.1/src/rosbag2_cpp/cache/cache_consumer.cpp:44)


```



播放包的打印信息：

```bash
➜  turtlebot3_demo git:(master) ✗ ros2 bag play rosbag2_2022_08_15-07_41_12/rosbag2_2022_08_15-07_41_12_0.db3                     
[INFO 1660520560.981193069] [rcl]: Found security directory: /home/kevin/turtlebot3_demo/test_keystore/enclaves (rcl_get_security_options_from_environment() at /tmp/binarydeb/ros-galactic-rcl-3.1.2/src/rcl/security.c:56)
1660520560.990039 [0]       ros2: using network interface wlp109s0 (udp/192.168.31.206) selected arbitrarily from: wlp109s0, anbox0, br-1f67ea43b642, br-fc22da249d2c, br-14287a178700, docker0
[INFO 1660520560.996492289] [rosbag2_storage]: Opened database 'rosbag2_2022_08_15-07_41_12/rosbag2_2022_08_15-07_41_12_0.db3' for READ_ONLY. (open() at /tmp/binarydeb/ros-galactic-rosbag2-storage-default-plugins-0.9.1/src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp:216)
[INFO 1660520561.000216516] [rosbag2_storage]: Opened database 'rosbag2_2022_08_15-07_41_12/rosbag2_2022_08_15-07_41_12_0.db3' for READ_ONLY. (open() at /tmp/binarydeb/ros-galactic-rosbag2-storage-default-plugins-0.9.1/src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp:216)

```



可以发现，只要多了一条下面的打印信息。说明消息传输已经加密了。

```bash
[rcl]: Found security directory: /home/kevin/turtlebot3_demo/test_keystore/enclaves (rcl_get_security_options_from_environment() at /tmp/binarydeb/ros-galactic-rcl-3.1.2/src/rcl/security.c:56)
```





**ros2 命令行工具是否可以用** 

在加密策略文件中加入下面内容在生成加密文件即可。

```xml
<policy version="0.2.0">
  <enclaves>
    <enclave path="/listener_talker">
      <profiles>
        <profile node="listener" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>chatter</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>
        <profile node="talker" ns="/">
          <services reply="ALLOW">
            <service>~/describe_parameters</service>
            <service>~/get_parameter_types</service>
            <service>~/get_parameters</service>
            <service>~/list_parameters</service>
            <service>~/set_parameters</service>
            <service>~/set_parameters_atomically</service>
          </services>
          <topics subscribe="ALLOW">
            <topic>parameter_events</topic>
          </topics>
          <topics publish="ALLOW">
            <topic>chatter</topic>
            <topic>parameter_events</topic>
            <topic>rosout</topic>
          </topics>
        </profile>

      </profiles>
    </enclave>
    <!-- 新增内容，支持ros2 命令行工具 -->
    <enclave path="/">
      <profiles>
        <profile node="_ros2cli" ns="/">
          <services reply="ALLOW" request="ALLOW">
            <service>*</service>
          </services>
          <topics publish="ALLOW" subscribe="ALLOW">
            <topic>*</topic>
          </topics>
          <actions call="ALLOW" execute="ALLOW">
            <action>*</action>
          </actions>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```



测试效果如下：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220815073749160.png)



**rqt 是否可以用**

设置了环境变量后，用rqt也可直接查看话题消息。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220815073945176.png)



**Rviz2 是否还可用**

也是可以用的。但需要设置相应的加密规则。

下面的测试场景来源于[https://github.com/ros-swg/turtlebot3_demo](https://github.com/ros-swg/turtlebot3_demo)

这是一个较为完整的机器人系统示例，里面包含了在加密情况下，运行**建图导航**功能。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220815082543905.png)







## 加密与未加密的数据区别

各个节点为了互相发现并建立节点会向`239.255.0.1`的`7400`端口发送数据包。下面是没有加密情况下用下面的命令获取的信息。

```bash
sudo tcpdump -X -i any udp port 7400
```



未加密的发现包：

```bash
20:52:34.439976 IP kevin-robot.60912 > 239.255.0.1.7400: UDP, length 272
	0x0000:  4500 012c 08e3 4000 2011 8067 c0a8 1fce  E..,..@....g....
	0x0010:  efff 0001 edf0 1ce8 0118 d1a0 5254 5053  ............RTPS
	0x0020:  0201 0110 bfb0 1001 9548 82dd d082 29bc  .........H....).
	0x0030:  0901 0800 f23c fa62 d566 ab48 1505 ec00  .....<.b.f.H....
	0x0040:  0000 1000 0000 0000 0001 00c2 0000 0000  ................
	0x0050:  0100 0000 0003 0000 2c00 1000 0a00 0000  ........,.......
	0x0060:  656e 636c 6176 653d 2f3b 0000 1500 0400  enclave=/;......
	0x0070:  0201 0000 1600 0400 0110 0000 0200 0800  ................
	0x0080:  0a00 0000 0000 0000 5000 1000 bfb0 1001  ........P.......
	0x0090:  9548 82dd d082 29bc 0000 01c1 5800 0400  .H....).....X...
	0x00a0:  3f0c 0000 0f00 0400 0000 0000 3100 1800  ?...........1...
	0x00b0:  0100 0000 7480 0000 0000 0000 0000 0000  ....t...........
	0x00c0:  0000 0000 c0a8 1fce 3200 1800 0100 0000  ........2.......
	0x00d0:  7480 0000 0000 0000 0000 0000 0000 0000  t...............
	0x00e0:  c0a8 1fce 0780 3800 0000 0000 2c00 0000  ......8.....,...
	0x00f0:  0000 0000 0000 0000 0000 0000 1e00 0000  ................
	0x0100:  6b65 7669 6e2d 726f 626f 742f 302e 382e  kevin-robot/0.8.
	0x0110:  302f 4c69 6e75 782f 4c69 6e75 7800 0000  0/Linux/Linux...
	0x0120:  1980 0400 0080 0600 0100 0000            ............

```



加密的发现包：

```bash
21:07:24.841490 IP kevin-robot.39365 > 239.255.0.1.7400: UDP, length 364
	0x0000:  4500 0188 bfc5 4000 2011 c928 c0a8 1fce  E.....@....(....
	0x0010:  efff 0001 99c5 1ce8 0174 d1fc 5254 5053  .........t..RTPS
	0x0020:  0201 0110 ef7c cb13 1fd0 198a bcdd 7ddd  .....|........}.
	0x0030:  0901 0800 f444 fa62 27e7 a9bd 1505 4801  .....D.b'.....H.
	0x0040:  0000 1000 0000 0000 0001 00c2 0000 0000  ................
	0x0050:  0100 0000 0003 0000 2c00 1000 0a00 0000  ........,.......
	0x0060:  656e 636c 6176 653d 2f3b 0000 1500 0400  enclave=/;......
	0x0070:  0201 0000 1600 0400 0110 0000 0200 0800  ................
	0x0080:  0a00 0000 0000 0000 5000 1000 ef7c cb13  ........P....|..
	0x0090:  1fd0 198a bcdd 7ddd 0000 01c1 5800 0400  ......}.....X...
	0x00a0:  3f0c ff0f 0510 0800 0700 0080 0600 0080  ?...............
	0x00b0:  0110 2000 1400 0000 4444 533a 4175 7468  ........DDS:Auth
	0x00c0:  3a50 4b49 2d44 483a 312e 3000 0000 0000  :PKI-DH:1.0.....
	0x00d0:  0000 0000 0210 2800 1b00 0000 4444 533a  ......(.....DDS:
	0x00e0:  4163 6365 7373 3a50 6572 6d69 7373 696f  Access:Permissio
	0x00f0:  6e73 3a31 2e30 0000 0000 0000 0000 0000  ns:1.0..........
	0x0100:  0f00 0400 0000 0000 3100 1800 0100 0000  ........1.......
	0x0110:  ec9c 0000 0000 0000 0000 0000 0000 0000  ................
	0x0120:  c0a8 1fce 3200 1800 0100 0000 ec9c 0000  ....2...........
	0x0130:  0000 0000 0000 0000 0000 0000 c0a8 1fce  ................
	0x0140:  0780 3800 0000 0000 2c00 0000 0000 0000  ..8.....,.......
	0x0150:  0000 0000 0000 0000 1e00 0000 6b65 7669  ............kevi
	0x0160:  6e2d 726f 626f 742f 302e 382e 302f 4c69  n-robot/0.8.0/Li
	0x0170:  6e75 782f 4c69 6e75 7800 0000 1980 0400  nux/Linux.......
	0x0180:  0080 0600 0100 0000                      ........
```

**可以发现**，加密后内容要多一些。

关于数据包，我参考官方文档并没有打印出来，在此就不作展示了。



参考：  

[https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Examine-Traffic.html](https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Examine-Traffic.html)





## `ros2 security`工具的使用



**生成 keystore**

```bash
ros2 security create_keystore demo_keystore
```



**生成keys and certificates** 

```bash
ros2 security create_enclave demo_keystore /talker_listener/talker
ros2 security create_enclave demo_keystore /talker_listener/listener
```

用上述命令生成的加密解密文件是比较通用的，没有指定加密解密特定的话题服务等。



**查看加密文件目录中的`enclaves`列表**

```bash
➜  demo_keystore ros2 security list_enclaves ./
/talker_listener/listener
/talker_listener/talker
```



**生成一个存储加密文件的目录**

```bash
ros2 security create_keystore test_keystore
```

注意，其中`test_keystore`为目录名称



**运行命令生成加密策略文件**

```bash
ros2 security generate_policy test_policy.xml
```



**`help`说明**

```bash
➜  turtlebot3_demo git:(master) ✗ ros2 security -h           
usage: ros2 security [-h]
                     Call `ros2 security <command> -h` for more detailed
                     usage. ...

Various security related sub-commands

optional arguments:
  -h, --help            show this help message and exit

Commands:
  create_enclave      Create enclave
  create_key          DEPRECATED: Create enclave. Use create_enclave instead
  create_keystore     Create keystore
  create_permission   Create permission
  generate_artifacts  Generate keys and permission files from a list of identities and policy files
  generate_policy     Generate XML policy file from ROS graph data
  list_enclaves       List enclaves in keystore
  list_keys           DEPRECATED: List enclaves in keystore. Use list_enclaves instead

  Call `ros2 security <command> -h` for more detailed usage.

```





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)





