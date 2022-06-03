---
title: ROS2开发环境搭建 #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: ROS2开发环境搭建
---

## 开发环境的建立

安装虚拟机和Ubuntu20

参考如下链接：

[https://fishros.com/d2lros2foxy/#/chapt2/2.1%E7%B3%BB%E7%BB%9F%E5%AE%89%E8%A3%85_%E8%99%9A%E6%8B%9F%E6%9C%BA%E7%89%88%E6%9C%AC ](https://fishros.com/d2lros2foxy/#/chapt2/2.1%E7%B3%BB%E7%BB%9F%E5%AE%89%E8%A3%85_%E8%99%9A%E6%8B%9F%E6%9C%BA%E7%89%88%E6%9C%AC)



安装VMware Tool

Ubuntu 主机

[https://kb.vmware.com/s/article/1022525](https://kb.vmware.com/s/article/1022525)

xxxxxxxxxx vcs export src >> tmp.reposBash



![image-20220601211729113](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220601211729113.png)



[https://www.epinv.com/post/5217.html](https://www.epinv.com/post/5217.html)



无法在host机和虚拟机间粘贴文件的问题：

[https://blog.csdn.net/sinat_28371057/article/details/109152328](https://blog.csdn.net/sinat_28371057/article/details/109152328)



安装ROS2

```Bash
wget http://fishros.com/install -O fishros && sudo bash fishros
```

按照脚本提示，安装ROS2 Galactic版本（若出现错误可重新运行脚本尝试）

[https://fishros.com/d2lros2foxy/#/chapt2/2.3ROS2%E7%9A%84%E5%AE%89%E8%A3%85](https://fishros.com/d2lros2foxy/#/chapt2/2.3ROS2%E7%9A%84%E5%AE%89%E8%A3%85)

安装vscode

[Download Visual Studio Code - Mac, Linux, Windows](https://code.visualstudio.com/Download)

```bash
sudo dpkg -i code_1.xx.0-1625728071_amd64.deb
```

在VScode中安装C++，Python，ROS2插件



github下载加速工具

[https://ghproxy.com/](https://ghproxy.com/)



添加下面的命令到~/.bashrc文件中

```bash
alias cb='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbns='colcon build'
alias cbpns='colcon build --packages-select '
alias git_pull_all_branch='git branch -r | grep -v "\->" | while read remote; do git branch --track "${remote#origin/}" "$remote"; done ; git fetch --all ; git pull --all'
```



扩展安装oh_myzsh

[https://www.jianshu.com/p/4fde9ae77922](https://www.jianshu.com/p/4fde9ae77922)



本文参考了《鱼香ROS》的文章。请关注“ 鱼香ROS ” 公众号获取更多学习材料。