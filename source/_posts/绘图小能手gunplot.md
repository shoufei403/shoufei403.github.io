---
title: 绘图小能手gunplot
categories: 实用工具
tags:
  - gunplot
abbrlink: 7ba7cdaf
date: 2022-08-21 11:30:08
---



下面的安装过程是在`ubuntu20.04`上进行的。



安装`gnuplot`需要依赖`lua5.2`。所以先安装`lua5.2`。



安装`lua5.2`

下载安装包

```bash
wget http://www.tecgraf.puc-rio.br/lua/ftp/lua-5.2.0.tar.gz
```

编译安装`lua5.2`

解压后进入源码目录

```bash
make linux
sudo make install
```

<!--more-->

安装`gnuplot`

`gnuplot`主页：[http://www.gnuplot.info/](http://www.gnuplot.info/)

`gnuplot`需要编译源码安装。首先到下面的网址下载源码。

下载网址：

[https://sourceforge.net/projects/gnuplot/files/gnuplot/5.4.3/](https://sourceforge.net/projects/gnuplot/files/gnuplot/5.4.3/)



编译安装`gnuplot`

```bash
./configure              # prepare Makefile and config.h
make                     # build the program and documentation
make check               # run unit tests to confirm successful build
sudo make install             # install the program and documentation
```



使用`gnuplot`

将下面的内容保存为`show.plt`。替换需要显示的数据文件，然后执行该脚本即可。

```bash
#!/usr/local/bin/gnuplot

set size ratio -1;
plot "test_path.txt" using 1:2:3 with labels offset -1,0.5 point pt 1 ps 1 lc rgb "blue" notitle
# plot "test2" u 1:2 t "Dawn" pt 1 ps .1
# plot "weird.path" u 1:2 smooth bezier
pause -1;

```

该脚本支持的数据存储形式为

```yaml
42.7901 36.8747 0
43.3355 36.8601 1
43.99 36.9039 2
```



显示效果：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220821110345658.png)

`gnuplot`还有很多功能，这样简单介绍一下。





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
