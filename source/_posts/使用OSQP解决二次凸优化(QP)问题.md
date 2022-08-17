---
title: 使用OSQP解决二次凸优化(QP)问题
categories: 路径规划
tags:
  - 二次凸优化
abbrlink: 3b3092bd
date: 2022-07-31 08:37:08
---

## 什么是二次凸优化问题
可以转化成满足如下方程的优化问题被称为二次凸优化(QP)问题。
```bash
min_x 0.5 * x'Px + q'x
s.t.  l <= Ax <= u
```
其中P是对称正定矩阵。所以目标函数的全局最小值就是其极小值。在二维的情况下，目标函数的图像类似下面的图。这里大概有一个印象就好。
![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200919162523378.png)
约束类型可以是等式约束和不等式约束。
当需要设置等式约束时可以将需要相等的行设置为`l[i] == u[i]` 。
单侧的不等式约束，可以将最小或最大侧设置成无穷小或无穷大。



<!--more-->
## 如何构造二次凸优化(QP)问题
这是一个比较大的问题。将很多实际的问题进行数学建模，然后转成凸优化问题。这样就能解了。这里仅说明一下这样的思路。




## 如何解二次凸优化(QP)问题
这里介绍如何使用`OSQP`库进行求解。
我已经将依赖的库合在一起了。可以在这里下载：

[https://github.com/shoufei403/OSQP](https://github.com/shoufei403/OSQP)  

使用`osqp`库和`osqp-eigen`库。`osqp-eigen`库是对`osqp`库的封装，其提供了更好用的`eigen`接口。

主要使用的接口如下：
```
    // set the initial data of the QP solver
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
    solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
    if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
    if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
    if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
    if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界
```

下面的实例来源于：

[https://ww2.mathworks.cn/help/optim/ug/quadprog.html?s_tid=srchtitle](https://ww2.mathworks.cn/help/optim/ug/quadprog.html?s_tid=srchtitle)  



- 具有线性约束的二次规划
  ![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200919174230134.png)

```c++
    hessian.resize(2,2);
    hessian.insert(0,0) = 1;
    hessian.insert(1,0) = -1;
    hessian.insert(0,1) = -1;
    hessian.insert(1,1) = 2;
    std::cout << "hessian:" << std::endl << hessian << std::endl;

    gradient.resize(2);
    gradient << -2, -6;
    
    std::cout << "gradient:" << std::endl << gradient << std::endl;

    linearMatrix.resize(3,2);
    linearMatrix.insert(0,0) = 1;
    linearMatrix.insert(0,1) = 1;
    linearMatrix.insert(1,0) = -1;
    linearMatrix.insert(1,1) = 2;
    linearMatrix.insert(2,0) = 2;
    linearMatrix.insert(2,1) = 1;
    std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

    lowerBound.resize(3);
    lowerBound << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

    upperBound.resize(3);
    upperBound << 2, 2, 3;
    std::cout << "upperBound:" << std::endl << upperBound << std::endl;

    int NumberOfVariables = 2; //A矩阵的列数
    int NumberOfConstraints = 3; //A矩阵的行数
```



- 具有线性等式约束的二次规划
  ![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200919174402564.png)

```c++
    hessian.resize(2,2);
    hessian.insert(0,0) = 1;
    hessian.insert(1,0) = -1;
    hessian.insert(0,1) = -1;
    hessian.insert(1,1) = 2;
    std::cout << "hessian:" << std::endl << hessian << std::endl;

    gradient.resize(2);
    gradient << -2, -6;
    
    std::cout << "gradient:" << std::endl << gradient << std::endl;

    linearMatrix.resize(1,2);
    linearMatrix.insert(0,0) = 1;
    linearMatrix.insert(0,1) = 1;
    std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

    lowerBound.resize(1);
    lowerBound << 0;
    std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

    upperBound.resize(1);
    upperBound << 0;
    std::cout << "upperBound:" << std::endl << upperBound << std::endl;

    int NumberOfVariables = 2; //A矩阵的列数
    int NumberOfConstraints = 1; //A矩阵的行数
```


- 具有线性约束和边界的二次最小化
  ![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200919174527819.png)
  该问题即包含了等式约束也包含了不等式约束。写成矩阵形式如下：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20220731204427.png)


```c++
    hessian.resize(3,3);
    hessian.insert(0,0) = 1;
    hessian.insert(1,0) = -1;
    hessian.insert(2,0) = 1;
    hessian.insert(0,1) = -1;
    hessian.insert(1,1) = 2;
    hessian.insert(2,1) = -2;
    hessian.insert(0,2) = 1;
    hessian.insert(1,2) = -2;
    hessian.insert(2,2) = 4;
    std::cout << "hessian:" << std::endl << hessian << std::endl;

    gradient.resize(3);
    gradient << 2, -3, 1;
    
    std::cout << "gradient:" << std::endl << gradient << std::endl;

    linearMatrix.resize(4,3);
    linearMatrix.insert(0,0) = 1;
    linearMatrix.insert(1,0) = 0;
    linearMatrix.insert(2,0) = 0;
    linearMatrix.insert(3,0) = 1;

    linearMatrix.insert(0,1) = 0;
    linearMatrix.insert(1,1) = 1;
    linearMatrix.insert(2,1) = 0;
    linearMatrix.insert(3,1) = 1;

    linearMatrix.insert(0,2) = 0;
    linearMatrix.insert(1,2) = 0;
    linearMatrix.insert(2,2) = 1;
    linearMatrix.insert(3,2) = 1;
    std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

    lowerBound.resize(4);
    lowerBound << 0, 0, 0, 0.5;
    std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

    upperBound.resize(4);
    upperBound << 1, 1, 1, 0.5;
    std::cout << "upperBound:" << std::endl << upperBound << std::endl;

    int NumberOfVariables = 3; //A矩阵的列数
    int NumberOfConstraints = 4; //A矩阵的行数
```



## 遇到的问题
1. 编译`osqp-eigen`库时报下面的错误：
 ```bash
CMake Error at cmake/OsqpEigenDependencies.cmake:12 (find_package):
  Could not find a configuration file for package "Eigen3" that is compatible
  with requested version "3.2.92".

  The following configuration files were considered but not accepted:

    /usr/lib/cmake/eigen3/Eigen3Config.cmake, version: unknown

Call Stack (most recent call first):
  CMakeLists.txt:63 (include)
 ```
解决措施：需要将原来旧的eigen库删掉，重新按照最新的eigen库。
```bash
sudo rm -rf /usr/include/eigen3
sudo rm -rf /usr/lib/cmake/eigen3
```
重新安装eigen，注意要安装到原来的位置`/usr/include`，不然catkin_make会报错。
```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
sudo make install
```

2. 编译osqp-eigen库时报`target_compile_features specified unknown feature cxx_std_14 for target`错误。
解决措施：将cmake升级到3.14版本后可以正常编译。但是` sudo apt remove cmake`时，把很多ros的库也删掉了，导致roscore都运行不了。
其实将`CMakeList.txt`文件更改一下就好了：
```bash
add_definitions(-std=c++14)  #添加这一行
#target_compile_features(${LIBRARY_TARGET_NAME} PUBLIC cxx_std_14)  #注释这一行
```

3. 使用osqp-eigen库时出现这样的问题：
```bash
In file included from /usr/local/include/OsqpEigen/OsqpEigen.h:10:0,
                 from /catkin_ws/src/MinimumSnap-Trajectory-Generation/waypoint_trajectory_generator/src/trajectory_generator_osqp.cpp:10:
/usr/local/include/OsqpEigen/Constants.hpp:12:18: fatal error: osqp.h: No such file or directory
```
这是因为头文件的包含路径有问题。按下图方式更改`osqp-eigen`库头文件，再重新编译安装。
![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200923114300308.png)
`Solver.tpp`文件中对`osqp`库头文件的引用要改成下面的方式

```bash
#include <osqp/auxil.h>
#include <osqp/scaling.h>
```

4. 运行程序链接osqp库时报错。`error while loading shared libraries: libosqp.so: cannot open shared object file: No such file or directory`
解决措施：需要添加相应的链接地址
```bash
sudo vim /etc/ld.so.conf
```
在里面添加一行
```bash
/usr/local/lib
```
最后再执行一下下面的语句
```bash
sudo  /sbin/ldconfig
```

5. 运行链接了`osqp`库和`OsqpEigen`的程序时，总是会报`Segmentation fault (core dumped)`。  
发现是`CMakeLists`的写法不同导致的。
原来自己的写法是：
```bash
set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native") //事实上是这一条不能加
target_link_libraries(trajectory_generator_node_test
   qdldl
   osqp
   OsqpEigen
)
```
官方的例子是这样写的。  
```bash
cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

project(OsqpEigen-Example)

find_package(OsqpEigen)
find_package(Eigen3)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

#MPCExample
add_executable(MPCExample src/MPCExample.cpp)
target_link_libraries(MPCExample OsqpEigen::OsqpEigen)

#Simple Example
add_executable(SimpleExample src/simpleqp_example.cpp)
target_link_libraries(SimpleExample OsqpEigen::OsqpEigen)
```
参照官方的例子写就正常了。







---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
