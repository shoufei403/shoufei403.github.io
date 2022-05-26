---
title: “从源码编译安装OpenCV” #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: OpenCV #分类
tags: [OpenCV] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: 从源码编译安装OpenCV
---

## 下载OpenCV源码

OpenCV的release页面

[Releases · opencv/opencv](https://github.com/opencv/opencv/releases)

选择相应的版本，下载源码

![OpenCV3.4.14](https://gitee.com/shoufei/blog_images/raw/master/20220118085754.png)

## 下载[opencv_contrib](https://github.com/opencv/opencv_contrib)源码

找到与opencv版本一致的tag

[https://github.com/opencv/opencv_contrib/tags](https://github.com/opencv/opencv_contrib/tags)

![opencv_contrib](https://gitee.com/shoufei/blog_images/raw/master/20220118085851.png)

## 编译安装opencv

1. 新建目录

```JavaScript
mkdir ~/opencv_build && cd ~/opencv_build
```

把下载好的opencv和opencv_contrib放到opencv_build目录下。

添加代理到编译过程中的下载链接

2. 对下列文件进行修改

```Apache
opencv_contrib-3.4.14/modules/xfeatures2d/cmake/download_boostdesc.cmake
opencv_contrib-3.4.14/modules/xfeatures2d/cmake/download_vgg.cmake
opencv_contrib-3.4.14/modules/face/CMakeLists.txt
```

在这些文件中的链接前加上`https://ghproxy.com/` 。这是一种代理下载的方式，加快下载速度。

加好的效果类似下图：

![](https://gitee.com/shoufei/blog_images/raw/master/20220118085941.png)



3. 编译

在opencv文件夹中新建build目录

```Apache
cd opencv-3.4.14
mkdir build
cd build
```

在build目录下执行下面的cmake命令。需要配置`OPENCV_EXTRA_MODULES_PATH` 参数。这个参数是opencv_contrib/modules的路径。

```SQL
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/home/kevin/opencv_build/opencv_contrib-3.4.14/modules \
    -D BUILD_EXAMPLES=ON ..
make -j8
sudo make install
```

编好的库会安装到`/usr/local/`下。



参考：

[https://opencv.org/](https://opencv.org/)

[https://www.jianshu.com/p/3c15a1ad3ec6](https://www.jianshu.com/p/3c15a1ad3ec6)

