---
title: Docker简明使用指南 #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: Docker #分类
tags: [Docker] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: 快速吧Docker用起来
---

对于`Docker`，我只想说早用早享受～







## 安装Docker

```SQL
# step 1: 安装必要的一些系统工具
sudo apt-get update
sudo apt-get -y install apt-transport-https ca-certificates curl software-properties-common

# step 2: 安装GPG证书
curl -fsSL http://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | sudo apt-key add -

# Step 3: 写入软件源信息
sudo add-apt-repository "deb [arch=amd64] http://mirrors.aliyun.com/docker-ce/linux/ubuntu $(lsb_release -cs) stable"

# Step 4: 更新并安装 Docker-CE
sudo apt-get -y update
sudo apt-get -y install docker-ce

# Step 5: 查看docker是否安装成功
docker version
```



## 安装共享显示接口组件

```Bash
sudo apt-get install x11-xserver-utils //确保安装了x11
```



## 拉取ubuntu20镜像

```Apache
docker pull shoufei/foxy:navigation2
```



## 显示GUI应用界面

```Nginx
 xhost +
```

先在**本机的命令窗口**运行这条命令，再在docker环境里运行GUI应用。



## 常用的Docker 命令
**创建镜像**

**以共享显示的方式创建容器**

```Groovy
sudo docker run -it \
  -v /etc/localtime:/etc/localtime:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/kevin/catkin_ws:/catkin_ws \
  -e DISPLAY=unix$DISPLAY \
  -e GDK_SCALE \
  -e GDK_DPI_SCALE \
  --name ubuntu20 \
  shoufei/foxy:navigation2 /bin/bash
```

注意：`-v /home/kevin/catkin_ws:/catkin_ws`

这条语句是将本机的目录映射到docker内部。其中`/home/kevin/catkin_ws`是本机目录，`/catkin_ws`是映射到docker后，docker中显示的目录。在docker中访问`/catkin_ws`即可访问本机的`/home/kevin/catkin_ws`目录。

**查看已经在运行的容器ID**

```C%2B%2B
//查看已经在运行的容器ID
sudo docker ps -a
```

**启动容器**

```Apache
sudo docker start eb9c37626c86 #eb9c37626c86为容器的id
```



**当启动了容器后可以用下面的命令多开容器命令窗口**

```Apache
sudo docker exec -it 31ced27e1684 /bin/bash  #31ced27e1684是容器的id
```



**停止容器**

```SQL
sudo docker stop 容器id
```

**删除启动的容器**

```Apache
sudo docker rm -f cf48d1718558
```

**删除镜像**

需要先把加载了镜像的容器停止，再删除。ubuntu是镜像仓库名，latest是tag。

```Nginx
sudo docker rmi ubuntu:latest
```



**commit 自己修改好的镜像**

注意：启动镜像时不要加--rm选项，加了的话退出后容器就会被删掉的。 

下面是一个示例： 

```Nginx
#启动容器
sudo docker run -it \
  -v /etc/localtime:/etc/localtime:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/kevin/catkin_ws:/catkin_ws \
  -e DISPLAY=unix$DISPLAY \
  -e GDK_SCALE \
  -e GDK_DPI_SCALE \
  --name kinetic \
  osrf/ros:kinetic-desktop-full /bin/bash

  #中间做了一些修改
  mkdir tes

  #ctrl+d退出

  #commit新的修改
  sudo docker commit -m "test " -a "Kevin Shou" 3f72a50a0372 kinetic:osqp
  #-m 写注释
  #-a 写作者
  #后面加想commit的容器ID
  #再后面是新镜像的命名

  #修改镜像名称，需保证名称有docker仓库的名称作为前缀
  sudo docker tag kinetic:osqp shoufei/kinetic:test

  #shoufei/kinetic是我在dockerhub上建的仓库名称。shoufei是用户名，kinetic是仓库名，test是仓库里的tag，用于区分不同的提交（commit）。
```



**本地登入docker**

```Nginx
sudo docker login
#登录自己的dockerhub的账户
```

**上传自己镜像**

```Bash
sudo docker push shoufei/kinetic:test
#shoufei/kinetic:test 为镜像的名称
```

**切换用户**

```Bash
sudo docker logout
```

**将当前用户加入docker群组中**

这样使用docker命令时就不用每次都加sudo了

```Bash
sudo usermod -aG docker $USER
```

**查询是否已经加入docker群组**

```Bash
id $USER
```



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。
