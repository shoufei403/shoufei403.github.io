---
title: ROS2中CMake编译选项的设置
categories: ROS2
tags:
  - ROS2
  - CMake
abbrlink: 5ce708c5
date: 2022-07-03 21:09:17
---

编译选项有很多，这里列出一些常用的编译选项设置，并说明作用。



## 指定使用的`C++`版本

```cmake
set(CMAKE_CXX_STANDARD 17)
```

可以根据需求设置11, 14, 17, 20等等`C++`版本。



## 设置编译选项

```cmake
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -g)
```

**告警选项：**

| 告警选项       | 作用描述                               |
| -------------- | -------------------------------------- |
| -Wall          | 允许发出gcc提供的所有有用的报警信息    |
| -Wextra        | 对所有合法但值得怀疑的表达式发出警告   |
| -Werror        | 把告警信息当做错误信息对待             |
| -Wpedantic      | 允许发出ANSI C标准所列的全部警告信息   |
| -w             | 关闭所有警告（不推荐使用）             |
| -Wfatal-errors | 遇到第一个错误就停止，减少查找错误时间 |



我们加上了告警都转成错误后常常会看到下面的错误信息。

```bash
error: unused parameter ‘test’ [-Werror=unused-parameter]
```
<!--more-->


```bash
error: variable 'begin' set but not used [-Werror=unused-but-set-variable]
```

有地方设置了变量但没有使用，可以加上下面的代码消除这个类型的错误。

```c++
#define UNUSED(x) (void)(x)
```



 **代码生成选项：**

| 代码生成选项                   | 选项作用描述                                                 |
| ------------------------------ | ------------------------------------------------------------ |
| -fPIC                          | 编译动态库时，要求产生与位置无关代码(Position-Independent Code)。也就是代码中不使用绝对地址，而使用相对地址，因此加载器可以将它加载到内存任意位置并执行。如果不使用-fPIC，产生的代码中包含绝对地址。加载器加载它时，要先重定位，重定位会修改代码段的内容，因此每个进程都生成这个代码段的一份拷贝。 |
| -fvisibility=default or hidden | 默认情况下，设置ELF镜像中符号的可见性为public或hidden。缺省值是default。hidden可以显著地提高链接和加载共享库的性能，生成更加优化的代码，提供近乎完美的API输出和防止符号碰撞。强烈建议在编译共享库的时候使用它。 |



**代码优化选项：**

| 优化选项                | 选项作用描述                                                 |
| ----------------------- | ------------------------------------------------------------ |
| -O0                     | 不优化。这是缺省值                                           |
| -O1                     | 尝试优化编译时间和可执行文件大小。                           |
| -O2                     | 尝试几乎全部的优化功能，但不会进行“空间换时间”的优化方法。   |
| -O3                     | 再打开一些优化选项：-finline-functions， -funswitch-loops 和 -fgcse-after-reload 。 |
| -O                      | 等同与-O1                                                    |
| -Os                     | 对生成文件大小进行优化。打开 -O2 开的全部选项，除了会那些增加文件大小的。 |
| -fomit-frame-pointer    | 去掉所有函数SFP（Stack Frame Pointer），即在函数调用时不保存栈帧指针SFP。可以提高程序运行速度， 代价是不能通过backtrace进行调试。 |
| -fno-omit-frame-pointer | 与-fno-omit-frame-pointer相反                                |

**注意：** -O1打开-fomit-frame-pointer选项

**注意：** 无特别需求，优化选项也可不单独设置，直接设置后面提到的**编译类型**即可



**`Debug`选项：**

| `Debug`选项 | 选项作用描述                                  |
| ----------- | --------------------------------------------- |
| -g          | 产生带有调试信息的目标代码                    |
| -ggdb       | 生成gdb专 用的调试信息，会有一些gdb专用的扩展 |
| -gdwarf-2   | 产生DWARF version2 的格式的调试信息           |

**注意：** 无特别需求，`Debug`选项也可不单独设置，直接设置后面提到的**编译类型**即可



**设置编译类型**

在编译命令中指定，如下所示：  

```bash
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo'
```



编译类型有如下四种：

| 编译类型       | 类型描述                                                     | 类似选项集                   |
| -------------- | ------------------------------------------------------------ | ---------------------------- |
| Debug          | 关闭了代码优化，增加了代码调试信息，提升了警告等级           | -O0 -g -Wall -D_DEBUG        |
| Release        | 程序达到最佳性能，运行速度最快，但丢失大量调试信息           | -O2 -DNDEBUG                 |
| RelWithDebInfo | 该模式会尽量按照`Release`的模式编译，但仍带有调试信息，是一个在速度和调试信息间的折中选择。 | -O0 -ggdb -DNDEBUG           |
| MinSizeRel     | 尽量减少执行文件的体积，一般用于嵌入式场景                   | -O1  -DNDEBUG -DMIN_SIZE_REL |



**补充说明：**

`DCMAKE_BUILD_TYPE`中的配置与`g++`编译选项之间的关系：  

`DCMAKE_BUILD_TYPE`是`Cmake`配置层面上的东西。最终编译类型的相关配置会转化到`g++`编译选项上。



参考：   

[https://blog.51cto.com/u_15169172/2710502](https://blog.51cto.com/u_15169172/2710502)







---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
