---
title: 如何白嫖GPU
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: '44009866'
date: 2023-09-30 20:33:16
---

这里介绍两个可以免费使用GPU的地方。



## 百度飞桨AI Studio

百度飞桨AI Studio提供的BML Codelab环境可以免费使用GPU。并且百度飞桨AI Studio中也开源了很多项目。只需要运行感兴趣的项目就会自动将该项目fork到自己的项目列表中。后面可在个人中心“我的项目”中查看。每天运行项目获得免费的GPU使用时长。



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230624100504987.png)



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230624100344543.png)





运行环境时会跳出可选的运行环境。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230623223658119.png)

**每日运行项目即送8点算力，可使用高级GPU V100 16GB算力运行16小时**。



另外一点要注意，当天运行项目自动领取的算力点数有效期只有一天。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230624093552799.png)


<!--more-->
当然如果需要使用百度飞桨AI Studio提供的GPU，首先得先注册一个账号。如果之前有百度账号可直接登陆的，没有的话可用下面的**邀请链接**注册。（注：成功注册会送我一些算力点数）

[https://aistudio.baidu.com/aistudio/newbie?invitation=1&sharedUserId=52600&sharedUserName=%E9%A6%96%E9%A3%9E1](https://aistudio.baidu.com/aistudio/newbie?invitation=1&sharedUserId=52600&sharedUserName=%E9%A6%96%E9%A3%9E1)





## Google cloab

访问网址：[https://colab.research.google.com/](https://colab.research.google.com/)

注意需要科学上网。



进入Google Colab中后，点击文件-新建笔记本，即可建立一个新的笔记本。

由于是Google的产品，在笔记本中去调用OpenAI的API是不受限制的。所以我用来测试一下调用OpenAI的API。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230626084801609.png)



修改笔记本配置，使之采用GPU。但是对于免费用户，GPU的类型只能选Tesla T4。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230626082111466.png)



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230626082010897.png)



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230626082337949.png)



新建一个code cell，用下面的命令可以查看GPU类型。

```bash
! /opt/bin/nvidia-smi 
```



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230626082535739.png)



另外，有些限制也需要注意一下。

免费用户笔记本最长可以持续运行 12 小时。断开链接后，运行产生的数据和状态都会清空。所以通常需要长久保存的数据和文件可以放在Google Drive中。然后在笔记本中挂载自己的Google Drive。



挂载方法：

1、新建一个code cell，写入下面的代码。

```python
from google.colab import drive 
drive.mount('/content/drive/') 
```

2、如果你是用Google账号登陆的Colab，运行代码后会提示让你授权。按提示操作即可。

3、授权好后，在笔记本左侧就能看到挂载的文件了。并且用右键点击，复制路径，就可以在代码中访问Google Drive中的文件了。





![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230626083755302.png)







Google Colab的使用教程可以参考下面的链接：

[http://www.tutorialspoint.com/google_colab/your_first_colab_notebook.htm](http://www.tutorialspoint.com/google_colab/your_first_colab_notebook.htm)





---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)