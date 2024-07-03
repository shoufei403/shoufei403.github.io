---
title: 云服务器上搭建gitlab和gitea的方法
categories: 实用技巧
tags:
  - 实用技巧
abbrlink: '47332190'
date: 2024-07-01 22:58:08
---

趁着618搞活动，以极低的价格买了一个云服务器。想着折腾点什么。发现可以自己搭一个`git`服务器。



开始的时候我试着装了装`Gitlab`。因为`Gitlab CE`版本是开源免费的，而且看到有些企业也在用。装上去运行后发现2核2G的云服务器直接卡死了，连`ssh`都登录不了。



后来又发现了`gitea`。这是一个轻量的`git`服务器。虽然没有`gitlab`那么多功能，但占用内存极少。在2核2G的云服务器上运行甚是欢快。



下面记录了安装和折腾过程。enjoy~~

<!--more-->

## 安装`Gitlab`

拉取gitlab的docker镜像

```bash
docker pull gitlab/gitlab-ce:latest
```



新建一个存放gitlab的目录

```bash
mkdir -p ~/docker/docker_gitlab
```



新建配置文件

```bash
cd ~/docker/docker_gitlab
vim docker-compose.yml
```

将下面的内容贴进去。其中`xx.xx.xx.xx`需要修改成自己机器的`ip`地址。

```yaml
version: '3.1'
services:
  gitlab:
    image: 'gitlab/gitlab-ce:latest'
    container_name: gitlab
    restart: always
    environment:
      GITLAB_OMNIBUS_CONFIG: | 
        external_url 'http://xx.xx.xx.xx:8929'
        gitlab_rails['gitlab_shell_ssh_port'] = 2224
    ports:
      - '8929:8929'
      - '2224:2224'
    volumes:
      - './config:/etc/gitlab'
      - './logs:/var/log/gitlab'
      - './data:/var/opt/gitlab'
```



启动`Gitlab`

```bash
sudo docker-compose up -d
```



关掉`Gitlab`

```bash
docker-compose down
```



重启`docker`

```bash
systemctl restart docker
```



查看启动日志

```bash
docker-compose logs -f
```



云服务器中设置`gitlab`端口放行

![image-20220619002611885](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220619002611885.png)



查看`gitlab`的`root`用户的初始密码

进入到`gitlab`容器内部

```bash
docker exec -it gitlab /bin/bash
```

查看`root`初始密码

```bash
cat /etc/gitlab/initial_root_password
```



参考：

5分钟搭建自己的代码托管平台gitlab

[https://zhuanlan.zhihu.com/p/387979095](https://zhuanlan.zhihu.com/p/387979095)



## 选择gitea来搭建git服务器

相比较与`gitlab`，`gitea`则轻量很多。在我买的服务器都不能流畅运行`gitlab`后，果断投入了`gitea`的怀抱。`gitea`的轻量和快速让我非常惊讶。下图展示了它的内存占用。

![image-20220623211719230](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220623211719230.png)

这可比动辄吃掉3个多G的gitlab要来的清爽。

但是针对大型企业还是会选择gitlab的。因为它是一个大而全的东西。gitea因为轻量，势必会缺少一些功能。但对于个人和小团体来说已经是够用了。gitea官方有一张与gitlab的功能对比表。可以查看下面的网址了解一下。

[https://docs.gitea.io/zh-cn/comparison/](https://docs.gitea.io/zh-cn/comparison/)





### 安装`gitea`

使用`docker`来安装

在服务器中新建一个存放`gitea`数据的目录。

```bash
mkdir -p /usr/local/docker/docker_gitea
```

新建一个`docker-compose.yml`文件

```yaml
version: "3"

networks:
  gitea:
    external: false

services:
  server:
    image: gitea/gitea:1.16.8
    container_name: gitea
    environment:
      - USER_UID=1000
      - USER_GID=1000
      - DB_TYPE=mysql
      - DB_HOST=db:3306
      - DB_NAME=gitea
      - DB_USER=gitea
      - DB_PASSWD=gitea
    restart: always
    networks:
      - gitea
    volumes:
      - ./gitea:/data
      - /etc/timezone:/etc/timezone:ro
      - /etc/localtime:/etc/localtime:ro
    ports:
      - "3000:3000"
      - "222:22"
    depends_on:
      - db

  db:
    image: mysql:8.0.29
    restart: always
    environment:
      - MYSQL_ROOT_PASSWORD=gitea
      - MYSQL_USER=gitea
      - MYSQL_PASSWORD=gitea
      - MYSQL_DATABASE=gitea
    networks:
      - gitea
    volumes:
      - ./mysql:/var/lib/mysql

```



使用下面的命令启动`gitea`

```bash
docker-compose up -d
```

更新`gitea`版本

只需要更新`docker-compose.yml`中的版本号。

```bash
image: gitea/gitea:1.16.8
```

查询gitea是否启动

```bash
docker-compose ps
```

查看gitea启动的log

```bash
docker-compose logs
```

需要在云服务器中的防火墙设置里打开相应的端口，此处是`3000`和`222`。

![image-20220623231704309](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220623231704309.png)





### 使用`nginx`设置反向代理

每次访问gitea服务都在url后面加3000端口确实不太好看，也不方便。所以这里使用`nginx`反向代理。



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

![image-20220623222612567](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220623222612567.png)



**配置反向代理**

`nginx` 配置文件都在`/etc/nginx/`目录下。  

修改`nginx`的配置文件`/etc/nginx/nginx.conf`，配置反向代理。  

添加下面的内容到`http`下。  

配置可参考：[https://www.runoob.com/w3cnote/nginx-setup-intro.html](https://www.runoob.com/w3cnote/nginx-setup-intro.html)

```bash
server {
        listen       80;
        server_name  example.cn;[此处填写域名]
        client_max_body_size 100M;
        location / {
                proxy_set_header Host $host;
                proxy_set_header X-Real-IP $remote_addr;
                proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
                proxy_pass    http://xx.xx.xx.xx:xxx;  #设置ip和端口
        }
        location ~ .*\.(js|css|png)$ {
                proxy_pass  http://xx.xx.xx.xx:xxx; ##设置ip和端口
        }
}
```



默认情况下，`nginx` 有一个上传单个文件大小限制，上传大于限制大小的文件会返回 **413** 错误，其大小限制默认值为 **1MB**。

报错信息：

```bash
Enumerating objects: 8, done.
Counting objects: 100% (8/8), done.
Delta compression using up to 12 threads
Compressing objects: 100% (4/4), done.
Writing objects: 100% (6/6), 1.90 MiB | 8.94 MiB/s, done.
Total 6 (delta 0), reused 0 (delta 0)
error: RPC failed; HTTP 413 curl 22 The requested URL returned error: 413
fatal: the remote end hung up unexpectedly
fatal: the remote end hung up unexpectedly
```

所以在上面的配置中加了`client_max_body_size 100M;`，以便修改这个限制。



检测配置文件语法是否正确

```bash
sudo /usr/sbin/nginx -t
```

重启`nginx`

```bash
sudo /usr/sbin/nginx
```

重新加载配置

```bash
sudo /usr/sbin/nginx -s reload
```

停止`nginx`

```bash
sudo /usr/sbin/nginx -s quit
```





**配置`gitea`服务器的域名**

打开`docker_gitea/gitea/gitea/conf/app.ini`文件。

修改`DOMAIN`、`SSH_DOMAIN`和`ROOT_URL`的值。

```yaml
[server]
APP_DATA_PATH    = /data/gitea
DOMAIN           = example.cn [这里填写域名]
SSH_DOMAIN       = example.cn [这里填写域名]
HTTP_PORT        = 3000
ROOT_URL         = http://example.cn/ [这里填写域名]
DISABLE_SSH      = false
SSH_PORT         = 22
SSH_LISTEN_PORT  = 22
LFS_START_SERVER = true
LFS_CONTENT_PATH = /data/git/lfs
LFS_JWT_SECRET   = XPTkmc3bXNfcDN3Nc5RmQcu-WZYmashiQpOAmlBJZ_M
OFFLINE_MODE     = false

```

修改好后就可以保存，然后重启`gitea`。

关闭`gitea`

```bash
docker-compose down
```

启动`gitea`

```bash
docker-compose up -d
```



**设置域名解析**

在腾讯购买域名后就可以使用`DNSPod`工具设置域名解析了。

![image-20220623233107472](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220623233107472.png)

这里设置的`git`是域名的前缀，可以以此来区分不同的服务。记录值中填写服务器的公网IP。



这时我们就可以通过域名直接访问`gitea`服务器了。



### 设置容器ssh直连

目前可以使用`http`来克隆和推送代码了。但是容器的`ssh`接口映射到了主机的非标准接口，所以无法使用`ssh`来克隆和推送代码。

官方提供的设置的说明，可参考下面的链接。

[https://docs.gitea.io/zh-cn/install-with-docker/#ssh-%E5%AE%B9%E5%99%A8%E7%9B%B4%E9%80%9A](https://docs.gitea.io/zh-cn/install-with-docker/#ssh-%E5%AE%B9%E5%99%A8%E7%9B%B4%E9%80%9A)



### 数据迁移

如果想把git仓库的数据迁移到另外的服务器中，需要把`gitea`打包好拷贝到新的服务器中。

![image-20220705085221952](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220705085221952.png)

同时mysql的备份需要先进入到mysql的docker环境中。

```
docker exec -it 2f986a21f76c /bin/bash  #2f986a21f76c 为docker container id
```

使用下面的命令备份数据库

```
mysqldump -uroot -pgitea --all-databases>all.sql
```

恢复数据库时，先启动gitea 的docker环境，让其生成mysql文件夹。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20240701224558292.png)

然后把之前备份好的all.sql文件拷贝到mysql文件夹中。

查看启动好的mysql docker id

```
docker ps -a
```

进入到mysql的docker环境中后进入到`/var/lib/mysql`目录中。

```
cd /var/lib/mysql
```

运行下面的命令恢复数据库

```
mysql --default-character-set=utf8mb4 -u 'root' -p <all.sql
```

提示输入密码时输入`gitea`。

最后在`gitea/gitea/conf/app.ini`文件中重新配置一下`gitea`的域名。就是下面的`SSH_DOMAIN`、`DOMAIN`和`ROOT_URL`。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20240701225431414.png)


参考[https://docs.gitea.cn/en-us/administration/backup-and-restore/](https://docs.gitea.cn/en-us/administration/backup-and-restore/)


## 搭建Drone  CI



`Drone`的官方文档：  

[https://docs.drone.io/](https://docs.drone.io/)



`Drone`是一个基于`Docker`技术的轻量CI/CD工具。它分为两个部分`server`和`runner`。`server`主要是对接`Git`服务器，获取仓库内容并解析`pipeline`文件。`runner`则运行`pipeline`文件中定义的内容。有多种`runner`可以使用。具体的介绍可以参考下面的网址：

[https://docs.drone.io/runner/overview/](https://docs.drone.io/runner/overview/)

![image-20220628232746375](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220628232746375.png)



比较常用的`runner`是`Docker Runner`。该`runner`会拉取相应的`docker`镜像，然后在镜像中运行定义的内容。



值得一提的是，`runner`是可以不和`server`运行在同一台服务器上的。多个`runner`也可以分别在不同的服务器上运行。



### 安装`drone server`

根据实际情况，修改下面的脚本并在云服务器上运行即可。



#### 获取`GITEA_CLIENT_ID`和`GITEA_CLIENT_SECRET`

在`gitea`中添加一个`OAuth2 Applications`，以便`Drone`可以使用它访问`gitea`。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220628234102378.png)

**需要注意的是**，`Redirect URI`中的域名需要添加`login`。并且如果域名申请的了SSL证书（可在DNSPod中操作），则使用`https`。



#### 获取`SHARED_SECRET`

使用下面的命令生成一个`shared_secret`

```bash
openssl rand -hex 16
```



####  使用脚本安装`drone server`

将上面获取到的内容修改到下面的脚本中。给脚本添加执行权限，然后运行即可。

```bash
#!/usr/bin/env bash

set -ex

GITEA_CLIENT_ID=0107e4c7-d1e8-4380-97f9-032f9eddc5e8
GITEA_CLIENT_SECRET=E6ibrZo8H3F3WNplLZITIkoJAVJAO2G99AxSQ2dhtPZZ
SHARED_SECRET=ac3963bc7225da1126185d1b88d5fddf
SERVER_HOST=drone.betainnovation.cn
SERVER_PROTOCOL=https

docker run \
  --volume=/var/run/docker.sock:/var/run/docker.sock \
  --volume=/var/lib/drone:/data \
  --env=DRONE_GITEA_SERVER=http://git.betainnovation.cn/ \
  --env=DRONE_GITEA_CLIENT_ID=${GITEA_CLIENT_ID} \
  --env=DRONE_GITEA_CLIENT_SECRET=${GITEA_CLIENT_SECRET} \
  --env=DRONE_AGENTS_ENABLED=true \
  --env=DRONE_RPC_SECRET=${SHARED_SECRET} \
  --env=DRONE_SERVER_HOST=${SERVER_HOST} \
  --env=DRONE_SERVER_PROTO=${SERVER_PROTOCOL} \
  --env=DRONE_TLS_AUTOCERT=true \
  --publish=80:80 \
  --publish=443:443 \
  --restart=always \
  --detach=true \
  --name=drone \
  drone/drone:2.12.1

```







### 安装`Docker Runner`

根据实际情况，修改下面的脚本并在云服务器上运行即可。



这里的`SHARED_SECRET`跟上面安装`server`使用的要保持一致。



`AGENT_SERVER_HOST`就是`drone server`的域名。



修改好下面的脚本内容，添加执行权限然后运行即可。

```bash
#!/usr/bin/env bash

set -ex

SHARED_SECRET=ac3963bc7225da1126185d1b88d5fddf
AGENT_SERVER_HOST=drone.betainnovation.cn
SERVER_PROTOCOL=https

docker run \
  --volume=/var/run/docker.sock:/var/run/docker.sock \
  --env=DRONE_RPC_PROTO=${SERVER_PROTOCOL} \
  --env=DRONE_RPC_HOST=${AGENT_SERVER_HOST} \
  --env=DRONE_RPC_SECRET=${SHARED_SECRET} \
  --env=DRONE_RUNNER_CAPACITY=2 \
  --env=DRONE_RUNNER_NAME=docker_runner_1 \
  --publish=3000:3000 \
  --restart=always \
  --detach=true \
  --name=drone-docker-runner-agent-01 \
  drone/drone-runner-docker:1.8.2
```



### 编写`pipeline`



参考网址：  

[https://docs.drone.io/quickstart/docker/](https://docs.drone.io/quickstart/docker/)



默认情况下，执行`pipeline`时会在前面加上`clone`操作。

![image-20220628232455981](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220628232455981.png)



`pipeline`示例：

下面的示例显示了如何使用飞书来发送构建信息。

```yaml
kind: pipeline
type: docker
name: default

steps:
- name: greeting
  image: alpine
  commands:
  - echo hello
  - echo world


- name: feishu Notice success
  pull: if-not-exists
  image: hb0730/drone-plugin-notice:1.0.2
  settings:
    debug: true
    notice_web_hok:      
        from_secret: feishu-robot-webhook
    notice_secret:
        from_secret: feishu-robot-secret
    notice_type: feishu
    message_type: markdown
    message_at_all: true
    message_title: Drone 构建通知
    message_content: |
      ### 构建信息
      > - 应用名称: [DRONE_REPO_NAME]
      > - 构建结果: 预发布成功 ✅
      > - 构建发起: [CI_COMMIT_AUTHOR_NAME]
      > - 持续时间: [CUSTOM_BUILD_CONSUMING]s

      构建日志: [点击查看详情]([DRONE_BUILD_LINK])        
  when:
    status: success
```



#### 设置飞书机器人

我们可在聊天群添加飞书机器人。

在群设置里，添加`BOTS`。



![image-20220628235812271](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220628235812271.png)



这里添加自定义机器人。这个类型的机器人可以使用`webhook`来交互。



将生成的`Webhook URL`设置到插件中即可。上面的示例是使用的`drone secret`。这个需要在`drone server`中配置。

![image-20220629000258397](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220629000258397.png)



#### feishu插件参数 plugin params

- `notice_access_token` (required) : 自定义机器人的 `webhok`
- `notice_type` (required) : 机器人类型: `dingtalk`,`feishu`
- `message_type` (required) : 消息类型: `text`,`markdown`
- `notice_secret` : 如果设置了`加签` , 可以把你的加签密钥填入此项完成加签操作。
- `message_at_all` : 是否`At`所有人
- `message_at_mobiles` : 你需要@的群成员的手机号，多个时用英文逗号(`,`)分隔 , 目前只支持 `dingtalk`
- `message_title` : 标题,只支持`markdown`
- `message_content` : 内容,支持占位符`[]` 替换，支持当前所有环境变量
- `debug` : debug模式，打印`env`等信息
- `custom_started` 开始时间环境变量,如:`DRONE_BUILD_STARTED`
- `custom_finished` 完成时间环境变量,如:`DRONE_BUILD_FINISHED`

- `CUSTOM_BUILD_CONSUMING` : 构建时间(秒)





## 使用Sonarqube扫描C/C++语言的项目

`SonarQube`的下载链接：

[https://www.sonarqube.org/downloads/](https://www.sonarqube.org/downloads/)



`SonarQube`提供了下面多个版本。个人用途的话可以使用开源免费的社区版本。但是社区版本是不支持C/C++扫描的。好在有大神做了一个插件来解决这个问题。

![image-20220711221722225](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220711221722225.png)



**基于`Docker`安装`SonarQube`**

1. 拉取`docker`镜像

```bash
docker pull sonarqube:8.9.9-community
```

**注意：**当前的长期支持版为`8.9.9`，所以这里拉取这个版本。



2. 新建一个`docker_sonarsqube`目录，并在里面创建一个`docker-compose.yaml`文件。写入下面的内容。

```yaml
version: '3.1'
services:
  db:
    image: postgres
    container_name: db
    ports:
      - 5432:5432
    networks:
      - sonarnet
    environment:
      POSTGRES_USER: sonar
      POSTGRES_PASSWORD: sonar
  sonarqube:
    image: sonarqube:8.9.9-community
    container_name: sonarqube
    depends_on:
      - db
    ports:
      - 9000:9000
    networks:
      - sonarnet
    volumes:
      - './logs:/opt/sonarqube/logs'
      - './extensions:/opt/sonarqube/extensions'
      - './data:/opt/sonarqube/data'
    environment:
      SONAR_JDBC_URL: jdbc:postgresql://db:5432/sonar
      SONAR_JDBC_USERNAME: sonar
      SONAR_JDBC_PASSWORD: sonar
networks:
  sonarnet:
    driver: bridge
```





3. 启动`sonarqube`

```bash
docker-compose up -d
```



查看启动过程的`log`

```bash
docker logs -f sonarqube
```



启动`sonarqube`时可能会报下面的错误。

![image-20220711231006529](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220711231006529.png)



**解决办法是**，在`/etc/sysctl.conf`文件中添加下面的内容。

```bash
vm.max_map_count=262144
```



运行下面的命令使设置生效

```bash
sudo sysctl -p
```



4. 访问`sonarqube`服务器

因为测试时是在自己电脑上安装的，所以可以访问`http://127.0.0.1:9000`来登录`sonarqube`服务器



默认账户和密码

```bash
账户： admin
密码： admin
```



登录后需要修改密码。



5. 配置界面为中文

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220711234330942.png)



6. 安装扫描`C/C++`的插件

插件的下载网址：

[https://hub.fastgit.xyz/SonarOpenCommunity/sonar-cxx/releases](https://hub.fastgit.xyz/SonarOpenCommunity/sonar-cxx/releases)

注意：这里放置的为`github`的代理网站，以便下载速度更快。原网址是[https://github.com/SonarOpenCommunity/sonar-cxx/releases](https://github.com/SonarOpenCommunity/sonar-cxx/releases)

亲测，`sonarqube-8.9.9`版本可以使用下面版本的插件。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220725083600917.png)

将下载好的`jar`包放置在目录中的`extensions/plugins`文件夹中。

重启`sonarqube`就能生效了。



然后更新一下`C++`的扫描规则



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220714075236059.png)

可以看到规则数是0。我们需要点击右上角的按钮新建一个`profile`。



![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220714075427664.png)



然后点击下面的标记的按钮添加规则。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220714075536775.png)



可以看到左边是规则的类别，右边是列出的规则。然后选择想要的规则并激活。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220714080010769.png)



批量激活插件中包含的规则

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220714080128394.png)



可以看到规则已经加好了。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220714080248107.png)



设置自定义的`profile`为默认。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220714080402844.png)





`sonarqube`默认是关闭`c++`文件的探测。需要如下图所示设置需要探测的`c++`文件后缀。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220716222522506.png)











7. 下载安装`SonarScanner`

下载网址：  

[https://docs.sonarqube.org/latest/analysis/scan/sonarscanner/](https://docs.sonarqube.org/latest/analysis/scan/sonarscanner/)



解压后移动到系统目录

```bash
mv sonar-scanner-4.7.0.2747-linux /opt/sonar-scanner
```



添加执行文件路径

```bash
echo 'export PATH="$PATH:/opt/sonar-scanner/bin"' >> ~/.bashrc
```

```bash
source ~/.bashrc
sonar-scanner -v
```



修改配置

```bash
vim /opt/sonar-scanner/conf/sonar-scanner.properties
```



如果`sonarqube`是安装在本地的，下面的`url`就写`localhost`。如果是安装在云服务器上就写服务器的ip。

```bash
sonar.host.url=http://localhost:9000
sonar.sourceEncoding=UTF-8
```



执行扫描

首先需要在`sonarqube`服务器上配置一个`project`。然后就能得到下面的命令。在需要扫描的目录下运行下面的命令即可。扫描结果可在服务器的网页上查看。

```bash
sonar-scanner \
  -Dsonar.projectKey=Cplusplus-analysis \
  -Dsonar.sources=. \
  -Dsonar.host.url=http://118.116.56.150:9000 \
  -Dsonar.login=495481cd66ab60d6fbd69586086f662dd7b4dd52
```

**注意：**第一次运行时需要下载东西会比较慢。



[https://techexpert.tips/sonarqube/sonarqube-scanner-installation-ubuntu-linux/](https://techexpert.tips/sonarqube/sonarqube-scanner-installation-ubuntu-linux/)





8. 静态代码分析



本机安装`cppcheck`

```bash
sudo apt-get update && sudo apt-get install cppcheck
```



使用`cppcheck`来检查代码

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



基于`docker`的`cppcheck`

```bash
docker pull neszt/cppcheck-docker

#在代码根目录运行
docker run -t -v $(pwd):/src neszt/cppcheck-docker
```



将`cppcheck`集成到`drone`中

在`.drone.yml`文件中添加下面的内容。

```yaml
- name: cppcheck
  pull: if-not-exists
  image: neszt/cppcheck-docker
  commands:
  - cppcheck --xml-version=2 . 2> cppcheck_report.xml

```



然后在`sonarqube`中设置报告的路径



![image-20220725083924971](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220725083924971.png)



**注意，**报告的名称要跟`.drone.yml`文件中保持一致。





参考：

[http://www.manongjc.com/detail/29-yxqbaowgctqxpxq.html](http://www.manongjc.com/detail/29-yxqbaowgctqxpxq.html)

[https://blog.csdn.net/weixin_44025546/article/details/111303183](https://blog.csdn.net/weixin_44025546/article/details/111303183)
