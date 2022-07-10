---
title: 在云服务器上基于docker安装jenkins
categories: 实用技巧
tags:
  - jenkins
abbrlink: c2ef42e4
date: 2022-07-10 21:50:16
---


`jenkins`是老牌的`CI/CD`工具。下面记录一下在云服务器上的安装过程。



## 基于`docker`安装`jenkins `

下面记录了如何在云服务器上安装`jenkins`。

新建一个`jenkins_docker`文件夹，在文件夹里新建一个`data`文件夹。并给`data`文件夹读写权限。

```bash
chmod -R a+w data/
```



新建一个`docker-compose.yml`文件。添加下面的内容：

```bash
version: "3.1"
services:
  jenkins:
    image: jenkins/jenkins:2.332.3-lts
    container_name: jenkins
    ports:
      - 8080:8080
      - 50000:50000
    volumes:
      - ./data/:/var/jenkins_home/
```

注意，这里是将`data`目录映射到了`docker`环境里。

<!--more-->

启动`jenkins`容器

在文件夹内运行

```bash
docker-compose up -d
```



停止`jenkins`容器

```bash
docker-compose down
```



更改`jenkins`插件下载源

打开`./data/hudson.model.UpdateCenter.xml`，替换下面的的内容。

```xml
# 修改数据卷中的hudson.model.UpdateCenter.xml文件
<?xml version='1.1' encoding='UTF-8'?>
<sites>
  <site>
    <id>default</id>
    <url>https://updates.jenkins.io/update-center.json</url>
  </site>
</sites>

# 将下载地址替换为http://mirror.esuni.jp/jenkins/updates/update-center.json
<?xml version='1.1' encoding='UTF-8'?>
<sites>
  <site>
    <id>default</id>
    <url>http://mirror.esuni.jp/jenkins/updates/update-center.json</url>
  </site>
</sites>

# 清华大学的插件源也可以
https://mirrors.tuna.tsinghua.edu.cn/jenkins/updates/update-center.json
```



查看运行`log`

```bash
docker logs -f jenkins
```

可以看到登录密钥

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220612231331322.png)

在浏览器中访问`主机ip:8080`就可以看到`jenkins`的登录页面。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220612232349652.png)

## 设置`jenkins`的反向代理

总是用云服务器的公网IP加端口号去访问`jenkins`，有些不方便了。我们可以使用`nginx`的方向代理来对应一个域名。当然域名得先买一个了。



**`ubuntu20`安装`nginx`** （云服务器里安装的ubuntu20.04）

```bash
sudo apt update
sudo apt install nginx
```



安装好后`nginx`会自动启动，可以用下面的命令查询`nginx`的状态

```bash
sudo systemctl status nginx
```

这时直接用浏览器访问服务器IP就可以看到`nginx`的欢迎界面了。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220623222612567.png)



修改`nginx`的配置文件`/etc/nginx/nginx.conf`。在`http`下添加下面的内容。注意这里监听的端口是8080。

```bash
server {
        listen       8080;
        server_name  jenkins.example.cn;[此处填写域名,可以加上jenkins以做区别]
        client_max_body_size 200M;
        location / {
                proxy_set_header Host $host;
                proxy_set_header X-Real-IP $remote_addr;
                proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
                proxy_pass    http://xx.xx.xx.xx:xxx;  #设置ip和端口
        }
        location ~ .*\.(js|css|png)$ {
                proxy_pass  http://xx.xx.xx.xx:xxx;  #设置ip和端口
        }
}
```



记得在云服务器里放行相应的端口号。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220625175255355.png)





**设置域名解析**

在购买域名后就可以使用`DNSPod`工具设置域名解析了。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220625175946220.png)

这里设置的`jenkins`是域名的前缀，可以以此来区分不同的服务。记录值中填写服务器的公网IP。



OK完事。这时就可以在浏览器中用`jenkins.域名`访问`jenkins`服务了。



`Jenkins`中文网址：  

[https://www.jenkins.io/zh/](https://www.jenkins.io/zh/)





---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
