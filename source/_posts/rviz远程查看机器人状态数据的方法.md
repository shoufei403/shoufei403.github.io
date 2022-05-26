

## ROS1环境

**匹配机器人和笔记本电脑的网络**

让笔记本电脑和机器人内部的机载电脑连上相同的局域网络。

下面分别配置电脑和机载电脑的`/etc/hosts `文件

使用`hostname` 分别查看笔记本电脑和机载电脑板的系统主机名。
使用`ifconfig` 分别查看笔记本电脑和机载电脑的`IP`地址。
使用`sudo vim /etc/hosts`打开电脑的`/etc/hosts`文件，添加机载电脑的主机名和`IP`地址。  
完成的效果如下：

```
127.0.0.1	localhost
127.0.1.1	kevin-Vostro-5568

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters

192.168.10.87 my_robot   #该条为添加内容
```
`my_robot `为机载电脑的主机名（由`hostname`命令获取）

同样的，在机载电脑上也修改`/etc/hosts` 文件，添加笔记本电脑的主机名和`IP`地址。效果如下：
```
127.0.0.1	localhost
127.0.1.1	my_robot
192.168.10.56 my_laptop  #该条为添加内容

```
`my_laptop` 为笔记本电脑的主机名（由`hostname`命令获取）



**告诉笔记本电脑`ROS_MASTER`的位置**

在笔记本电脑上打开一个命令窗口，输入下面的命令配置`ROS_MASTER_URI`。

```bash
export ROS_MASTER_URI="http://192.168.10.87:11311"       #其中192.168.10.87为机器人电脑的ip地址
```

这样运行完后，在该命令窗口打开`Rviz`就可以接收到机器人端的状态数据并在`Rviz`上显示了。



PS：如果想直观地远程打开机器人电脑中的文件，可以使用`Ubuntu`文件窗口(Connect to Server)访问机载电脑上的文件。

![Screenshot from 2020-10-22 17-00-51](https://gitee.com/shoufei/markdown_picture/raw/master/img/Screenshot%20from%202020-10-22%2017-00-51.png)

`Server address` 的地址按下面的形式来写：

```bash
sftp://root@192.168.1.15/usr
```





## ROS2环境

`ROS2`中通信的中间件与`ROS1`不一样了。`ROS2`的通信网络有自探索功能，不需要`ROS1`环境下的那些配置了。  

但是需要确保笔记本电脑上的`ROS_DOMAIN_ID`和机器人电脑上的`ROS_DOMAIN_ID`要一致。  

`ROS_DOMAIN_ID`可以用来区别同一个局域网络中的不同机器人。只用`ROS_DOMAIN_ID`相同的机器人才能互相通信。  

默认情况下，`ROS_DOMAIN_ID`是`0`。 

因为一些原因，`ROS_DOMAIN_ID`通常在`1～101`选择是比较保险的。

更多关于`ROS_DOMAIN_ID`的解释，请参考下面的文章：

[https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html](https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html)



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![](https://gitee.com/shoufei/blog_images/raw/master/shoufei_qr.jpg)

