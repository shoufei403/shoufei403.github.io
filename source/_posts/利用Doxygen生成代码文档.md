---
title: 利用Doxygen生成代码文档
categories: 实用工具
tags:
  - Doxygen
abbrlink: cd88f09
date: 2022-08-21 11:29:08
---

`Doxygen`是一个代码文档生成工具。它从代码文件中提取注释并可生成多种文档形式。如：网页文档HTML，RTF (MS-Word)，PDF等等。同时也可生成函数之间的调用和文件的依赖关系图表。



`Doxygen`除了支持`C++`语言外还支持C, Objective-C, C#, PHP, Java, Python, IDL (Corba, Microsoft, and UNO/OpenOffice flavors)，甚至它也支持硬件描述语言VHDL。



## doxygen的安装



1. 使用`apt`安装`doxygen`

```bash
sudo apt install doxygen
```

<!--more-->

2. 使用最新版的二进制安装（该种方式想对于第一种，可安装最新的版本）

`doxygen`的下载页面：

[https://www.doxygen.nl/download.html](https://www.doxygen.nl/download.html)

找到下面图片所示，下载二进制包。

![image-20220816083732322](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220816083732322.png)

可以看到，该二进制包是在`Ubuntu 20.04`环境下编译的，可能不适用于其他版本的系统。

解压二进制包后，进入包文件夹，使用下面的命令安装。

```bash
sudo make install
```

`makefile`中没有安装`doxywizard`。

我们可以手动拷贝到`/usr/local/bin/`中。

```bash
cd doxygen-1.9.4/bin
sudo cp doxywizard /usr/local/bin/
```



`Doxywizard`是一个`GUI`应用。可以用它来生成`Doxygen`的配置文件。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220816080803347.png)



3. 安装`graphviz`

`Graphviz`是开源的图形可视化软件。它可以将结构化的信息以图表的形式显示出来。`doxygen`可以调用`Graphviz`显示函数的调用关系。

```bash
sudo apt install graphviz
```



4. `htmlhelp`说明

`htmlhelp`是一个可以将`html`网页文件生成一个独立的`chm`文件的软件工具。但它目前只能运行在`windows`环境下。

如果需要生成`chm`文件，可将`doxygen`生成的网页文件拷贝至`windows`环境下，然后用`htmlhelp`来生成`chm`文件。



## 注释和文档效果

1. 头文件中添加如下函数注释。

```c++
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
```

上面的注释方式是采用`Javadoc`风格的。其实还有其他的风格。可以查看下面的网址了解：

[https://www.doxygen.nl/manual/docblocks.html#cppblock](https://www.doxygen.nl/manual/docblocks.html#cppblock)



文档中对应的显示效果如下。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220816225708898.png)



另外一个示例：

```c++
  /**
  * @brief Construct the TebConfig using default values.
  * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
  *	     the default variables will be overwritten: \n
  *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
  * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
  * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are
  * 	     set in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg. \n
  * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
  * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
  * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
  * 		<b>TebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
  */
  TebConfig()
  {

  }
```

`\b`  可以加粗后面的关键词，`\e`表示后面的词进行斜体显示，`<b> ...</b> ` 可将其中的内容加粗显示。`\n` 则是显式添加回车。



文档上的显示效果：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220819071306546.png)



再一个示例：

```c++
      /**
       * A pure virtual member.
       * @see testMe()
       * @param c1 the first argument.
       * @param c2 the second argument.
       */
       virtual void testMeToo(char c1,char c2) = 0;
```



显示效果：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220819082153581.png)



**注意**，注释中添加的关键字。

`@brief`  表示后面的内容是对函数功能的描述

`@warning` 一些警告信息

`@param` 传入参数的说明

`@return` 函数返回结果的说明

`@see` 方便跳转相关联的函数



**另外注意，**函数的注释放在头文件和源文件中效果是等同的。



2. 项目中的`markdown`文档会生成相应的页面

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220818083738946.png)



3. 对类成员的注释

```c++
  std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame; //!< Global planning frame
  std::string node_name; //!< node name used for parameter event callback
```



文档显示效果：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220819071156912.png)



4. 注释一个结构体

```c++
  //! Trajectory related parameters
  struct Trajectory
  {
    double teb_autosize; //!< Enable automatic resizing of the trajectory w.r.t to the temporal resolution (recommended)
	...
    int control_look_ahead_poses; //! Index of the pose used to extract the velocity command
  } trajectory; //!< Trajectory related parameters
```

结构体上面的`  //! Trajectory related parameters`是对结构体的描述。

下面的`//!< Trajectory related parameters`是对声明的类成员的描述。注意与上面符号的区别，这里多了一个`<`。其实`<`说明了注释的方向。



5. 注释一个类

```c++
/**
 * @class TebConfig
 * @brief Config class for the teb_local_planner and its components.
 */
class TebConfig
{
    ...
}
```







## 生成文档



### 生成配置文件

用下面的命令生成配置模板文件

```bash
doxygen -g
```

运行完后默认会生成一个名为`Doxyfile`的配置文件。



然后就可以根据需求手动修改配置文件了。



当然我们也可以基于图形界面来修改该文件。



运行`doxywizard`

```bash
doxywizard
```



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220818083044972.png)



然后点击`file`->`open`打开`Doxyfile`配置文件。



或者直接使用

```bash
doxywizard Doxyfile
```





### 常用的参数配置

1. 打开调用关系图

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220819081716499.png)



显示效果如下：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220819081739942.png)

上图显示了该函数调用了哪些函数，然后又被什么函数调用了。



2. 当`JAVADOC_AUTOBRIEF`参数设置为`YES`时，会将下面的注释内容直接当成简介描述。

```c++
/**
 *  A test class. A more elaborate class description.
 */
```

如果设置成`NO`的话，则需要添加`@brief`显式标记。

```c+
/**
 *  @brief A test class. A more elaborate class description.
 */
```



3. 如果希望生成的文档中包含源码，则需要如下配置
4. ![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220819083855598.png)





### 生成文档

在具有`Doxyfile`配置文件的目录下运行`doxygen`即可生成文档。

```bash
doxygen
```



也可以在`doxywizard`里点击运行`doxygen`来生成文档。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220818083420939.png)







---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)

