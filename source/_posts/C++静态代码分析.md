---
title: C++静态代码分析
categories: C/C++
tags:
  - C/C++
abbrlink: c109de2a
date: 2022-07-31 08:38:08
---

这里记录一下使用`cppcheck`进行`C++`代码静态检测的方法和步骤。



**本机安装`cppcheck`**

```bash
sudo apt-get update && sudo apt-get install cppcheck
```



**使用`cppcheck`来检查代码**

新建一个目录，并在目录中加入如下内容的`cpp`文件，用于测试静态代码分析工具。

测试代码

```c++
#include <iostream>

using namespace std;

int test_fun()
{
  int a;
  return a;
}


int main(int argc, char* argv[]) {
  int num = argc - 1;

  int * a = nullptr; // intentional mistake
  *a = 9;

  if (num = 0) {
    cout << "No arguments provided\n";
  } else if (num == 0) {  // intentional mistake
    cout << "1 argument provided\n";
  } else if (num == 2) {
    cout << "2 arguments provided\n";
  } else {
    cout << num << " arguments provided\n";
  }
  if (argv != 0) {
    cout << "argv not null\n";;  // intentional extra-semicolon
  }
  if (argv == nullptr) {
    return **argv;  // intentional nullptr dereference
  }

  return 0;
}
```

<!--more-->

使用`cppcheck`检测效果如下：

```bash
➜  test_git git:(master) ✗ cppcheck ./src                                                                                                                                                                                                      
Checking src/main.cpp ...
src/main.cpp:8:2: error: Null pointer dereference: a [nullPointer]
*a = 9;
 ^
src/main.cpp:7:11: note: Assignment 'a=nullptr', assigned value is 0
int * a = nullptr;
          ^
src/main.cpp:8:2: note: Null pointer dereference
*a = 9;
 ^
➜  test_git git:(master) ✗ ament_cppcheck ./src                                                                                                                                                                                              
[src/main.cpp:8]: (error: nullPointer) Null pointer dereference: a
1 errors

```



**基于`docker`的`cppcheck `**

```bash
docker pull neszt/cppcheck-docker

#在代码根目录运行
docker run -t -v $(pwd):/src neszt/cppcheck-docker
```



**在`vscode`中进行静态代码检测**

在安装了`ROS2`后，可使用下面的命令安装` ament linters `。

```bash
sudo apt-get install ros-$ROS_DISTRO-ament-lint
```



运行完后，系统目录`/opt/ros/galactic/bin`下就会被安装下图所示的文件。

其中的`ament_cppcheck`即可用于`C++`静态代码检测。

![image-20220727215654381](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220727215654381.png)



可直接运行`ament_cppcheck`命令。效果与之前的`cppcheck`类似。

```bash
➜  test_git git:(master) ✗ ament_cppcheck src/                git:(master|✚1…3 
[src/main.cpp:19]: (error: nullPointer) Null pointer dereference: a
[src/main.cpp:8]: (error: uninitvar) Uninitialized variable: a
2 errors

```



这里我们将其加入到`vscode`的`tasks.json`文件中。这样就可以直接在`vscode`中直接运行静态代码检测任务了。

```json
        {
            "label": "cppcheck",
            "detail": "Run static code checker cppcheck.",
            "type": "shell",
            "command": "ament_cppcheck src/",
            "presentation": {
                "panel": "dedicated",
                "reveal": "silent",
                "clear": true
            },
            "problemMatcher": [
                {
                    "owner": "cppcheck",
                    "source": "cppcheck",
                    "pattern": [
                        {
                            "regexp": "^\\[(.+):(\\d+)\\]:\\s+(.+)$",
                            "file": 1,
                            "line": 2,
                            "message": 3
                        }
                    ]
                }
            ]
        },
```



按`Shift+Ctrl+P`打开命令面板，输入`task`并回车即可看到任务列表。



完整的`tasks.json`文件还包含代码格式化，代码格式检测以及其他有用的任务。文件过长，可到下面的链接中查看。

[https://github.com/shoufei403/ros2_galactic_ws/blob/master/.vscode/tasks.json](https://github.com/shoufei403/ros2_galactic_ws/blob/master/.vscode/tasks.json)





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)