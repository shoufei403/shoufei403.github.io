---
title: 使用Hexo+Github+腾讯云+Netlify搭建个人博客
categories: 实用技巧
tags:
  - Hexo
  - 博客
abbrlink: fd522dc
date: 2022-06-19 22:38:16
---


## 安装Hexo

要使用`Hexo`必须先安装`Git`和`Node.js`。本文是在`Ubuntu20`环境下进行操作的。使用其它系统也可以将下面的操作作为参考。

### 安装Git

```bash
sudo apt-get install git-core
```



### 安装Node.js

推荐安装当前最新版。不同的`Hexo`版本依赖不同版本的`Node.js`。下面是版本对照表。

![image-20220603230418480](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220603230418480.png)

这里建议安装`Node.js`最新稳定版。

`Node.js`官方下载网址：

[https://nodejs.org/en/download/](https://nodejs.org/en/download/)

这里以二进制安装方式进行，也可使用源码编译安装。

按图示下载好二进制文件包。

![image-20220603230636874](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220603230636874.png)

按照下面的方式操作即可安装好。

```bash
tar xvf node-v16.15.1-linux-x64.tar.xz
mkdir /data
mv node-v16.14.2-linux-x64 /data/nodejs
ln -s /data/nodejs/bin/* /usr/bin/
npm install yarn

echo "PATH=$PATH:/data/nodejs/bin"  >> /etc/profile
source /etc/profile
```

<!--more-->

###  安装Hexo

```bash
npm install -g hexo-cli
```



## 创建博客

新建博客目录

```bash
hexo init blog
```

安装node依赖

```bash
cd blog
npm install
```

本地启动博客服务器

```bash
hexo server
```

如果有出现下面的报错信息，可更改一下端口号。

![image-20220604201446312](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604201446312.png)

在博客目录内的`_config.yml`文件内添加下面的代码来更改hexo-server运行时的端口号：

```bash
server:
  port: 4001 #端口号随意，默认是4000
  compress: true
  header: true

```

更改后的效果：

![image-20220604202335598](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604202335598.png)

现在用浏览器访问`http://localhost:4001`就可以看到初始的博客了。

![image-20220604202453433](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604202453433.png)



## 写博客文章

**创建博客文章**

```bash
hexo new "My first blog"
```

运行上面的命令将创建一片名为`My first blog`的博客文章。

![image-20220604202853879](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604202853879.png)

可以看到文章保存到了`source/_posts`目录下了。并且文章是以`markdown`的形式保存的。

其实也可以自己手动在`source/_posts`目录下新建`markdown`文件。当我们编译博客目录时，这些`markdown`文件将会生成对应的网页文件。



当我们用命令创建一个新的博客时，其实是用了一个默认模板来创建一个新的`markdown`文件。这个模板是保存在`scaffolds`目录下的。

![image-20220604203356371](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604203356371.png)

其中`post.md`就是博客文章的模板。一个常见的模板是这样的。

`post `模板

```yaml
---
title: {{ title }}
date: {{ date }}
categories: 
tags: 
description: 
photos:
- 
---

```

`title`: 文章标题

`date`: 文章的生成时间

`categories`: 文章分类，这个分类会在网站上统计

`tags`: 文章的标签

`description`: 文章的概要

`photos`: 文章的封面

**注意**，每一篇文章的头部都需要有一个这样的开头。`description`和`photos`是可以不加的。

一个简单的示例：

```yaml
---
title: Costmap是什么？ #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2,Costmap] #文章标签，可空，多标签请用格式，注意:后面有个空格
# description: 详细介绍了costmap
---
```



**显示摘要**

在文章中添加下面的内容

```yaml
<!--more-->
```

**注意**，摘要显示和文章描述显示只能**选择一个**。如果要显示文章描述，则文章的开头要加上`description`。若想显示摘要，则文章开头不可加`description`。不然只会显示`description`。然后把`<!--more-->`加入到文章中。`<!--more-->`之前即为摘要。效果如下：

![image-20220603215928327](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220603215928327.png)

如果想在摘要中显示图片可以将`<!--more-->`放在图片链接后面。比较合适的图片尺寸为`1600x900`。

![image-20220604204902415](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604204902415.png)



**更改博客主题**

因为博客文章是用`markdown`来写的，所以我们可以任意更换文章呈现的主题。

这里使用`NexT `来作为示范。

将`NexT`主题下载到`themes`目录。

```bash
cd themes
git clone https://github.com/theme-next/hexo-theme-next
```

然后在`_config.yml`文件中配置需要使用的主题。

```yaml
theme: hexo-theme-next
```

主题也是可以配置的。详细的配置和使用说明请查看官方文档。

NexT 使用文档:

[http://theme-next.iissnan.com/getting-started.html](http://theme-next.iissnan.com/getting-started.html)



**设置头像**

编辑 `themes/hexo-theme-next/_config.yml`：

```bash
avatar: <avatar-url>
```

**添加标签页面**

- 新建页面：

  ```bash
  hexo new page tags
  ```

- 设置页面（编辑 `source/tags/index.md`）：

  ```bash
  ---
  type: "tags"
  comments: false
  ---
  ```

- 修改菜单（编辑 `themes/hexo-theme-next/_config.yml`）：

  ```bash
  menu:
    tags: /tags
  ```

**添加分类页面**

- 新建页面：

  ```bash
  hexo new page categories
  ```

- 设置页面（编辑 `source/categories/index.md`）：

  ```bash
  ---
  type: "categories"
  comments: false
  ---
  ```

- 修改菜单（编辑 `themes/hexo-theme-next/_config.yml`）：

  ```bash
  menu:
    tags: /categories
  ```

**添加 about 页面**

- 新建页面：

  ```bash
  hexo new page about
  ```

- 设置页面（编辑 `source/about/index.md`）

- 修改菜单（编辑 `themes/hexo-theme-next/_config.yml`）：

  ```bash
  menu:
    about: /about
  ```



**给博客安装搜索插件**

使用本地搜索，按以下步骤配置：

- 安装 `hexo-generator-searchdb` 插件：

  ```bash
  npm install hexo-generator-searchdb --save
  ```

- 编辑 `_config.yml`：

  ```bash
  search:
    path: search.xml
    field: post
    format: html
    limit: 10000
  ```

- 编辑 `themes/hexo-theme-next/_config.yml`：

  ```bash
  # Local search
  local_search:
    enable: true
  ```



**给博客文章设置永久链接**

`Hexo` 默认文章链接生成规则是按照年、月、日、标题来生成的。一旦文章标题或者发布时间被修改，`URL` 就会发生变化，之前文章地址也会不可访问。而且 `URL` 层级很深，不利于分享和搜索引擎收录。



如果文章标题中有中文，`URL` 被转码后会很长，比如：`https://www.shoufei.xyz/2022/06/12/%E4%B8%8B%E8%BD%BDturtlebot4%E6%BA%90%E7%A0%81%E6%9D%A5%E5%AD%A6%E4%B9%A0/`。



我们可以使用 `hexo-abbrlink`插件为每篇文章生成一个唯一字符串，这个字符串ID不受文章标题和发布时间的影响，比如：`https://www.shoufei.xyz/p/ae5fa866`。



1. 安装插件

点击即可访问插件源码地址 [hexo-abbrlink](https://www.github.com/rozbo/hexo-abbrlink) 。

在博客目录下运行下面的命令安装插件

```bash
npm install hexo-abbrlink --save
```



2. 配置

修改博客根目录配置文件 `_config.yml` 的 `permalink`：

```bash
# permalink: :year/:month/:day/:title/
permalink: p/:abbrlink.html  # p 是自定义的前缀
abbrlink:
    alg: crc32   #算法： crc16(default) and crc32
    rep: hex     #进制： dec(default) and hex
```

不同算法和进制生成不同的字符串ID：

```bash
crc16 & hex
https://post.zz173.com/posts/66c8.html
crc16 & dec
https://post.zz173.com/posts/65535.html

crc32 & hex
https://post.zz173.com/posts/8ddf18fb.html
crc32 & dec
https://post.zz173.com/posts/1690090958.html
```

3. 验证

先清理下本地的文件 `hexo clean`，然后重新生成 `hexo g`，启动博客 `hexo s`。该插件会在每篇文章的开头增加内容：

```bash
abbrlink: df27ccfb
```

这个字符串就是这篇文章的唯一标识，无论修改标题还是发布文章都不会改变。



参考：

[https://zhuanlan.zhihu.com/p/134492757](https://zhuanlan.zhihu.com/p/134492757)



## 部署博客

目前我学习到两种部署博客网站的方式。一种是使用`Github Pages`来托管博客网站，一种是通过`Netlify`来托管。

### 使用Github Pages

安装部署插件

```bash
cnpm install --save hexo-deployer-git
```

在github上新建一个仓库，用于存放网页内容。**注意**，设定的仓库名称必须是`用户名+github.io`的形式。

![image-20220604211646006](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604211646006.png)



然后修改_config.yaml中的配置，加上目标仓库的地址。

![image-20220604211727378](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604211727378.png)



运行`hexo d`后其中的`publish`目录中的内容将被推送到设定的远程仓库中。

**注意**，每次推送都是完全覆盖。

**注意**，这里`repo`的地址需要用`ssh`的仓库地址。

**注意**，`repo`必须保持为公开状态。

这时通过访问`用户名+github.io`就能看到你的博客了。



**常用命令**

下面命令的顺序就是一个部署流程。

```bash
hexo new "postName" #新建文章
hexo new page "pageName" #新建页面
hexo clean #清除hexo生成的内容，建议生成前先clean一下，避免出现异常
hexo generate #生成静态页面至public目录
hexo server #开启预览访问端口（默认端口4000，'ctrl + c'关闭server）
hexo deploy #将博客内容部署到GitHub
```

可简写为：

```bash
hexo n == hexo new
hexo g == hexo generate
hexo s == hexo server
hexo d == hexo deploy
```

参考：

[https://hexo.io/zh-cn/docs/github-pages](https://hexo.io/zh-cn/docs/github-pages)



### 使用Netlify

[Netlify](https://www.netlify.com/) 是一个提供网络托管的综合平台。它集持续集成（CI），CDN 自定义域名， HTTPS ，持续部署（CD）等诸多功能于一身。

首先，还是在`Github`上新建一个仓库，只是这次对仓库的名称没有要求了。然后用`git`工具将博客目录的文件推送到该仓库。

**注意**，不需要使用`hexo g`来生成博客网页。这一步我们将在[Netlify](https://www.netlify.com/) 上完成。所以推送的东西**不应该**包含运行`hexo g`命令生成的文件。



然后，在[Netlify](https://www.netlify.com/) 上**注册**一个帐号。最好直接使用`Github`帐号。



**并添加好维护博客内容的仓库。**

![image-20220604215401059](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604215401059.png)



**设置编译命令**

![image-20220604215454097](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604215454097.png)

**设置部署的分支**

![image-20220604215539900](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604215539900.png)



**修改默认域名**

[Netlify](https://www.netlify.com/) 会默认为我们的博客生成一个域名，但是这个域名比较复杂。我们可以自定义这个域名。

![image-20220604220113403](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604220113403.png)

**绑定自己的域名**

首先需要购买一个自己的域名。可以购买域名的网站如下：

[https://wanwang.aliyun.com/domain/](https://wanwang.aliyun.com/domain/)

[https://dnspod.cloud.tencent.com/](https://dnspod.cloud.tencent.com/)

[https://www.godaddy.com/zh-sg](https://www.godaddy.com/zh-sg)

个人博客用的域名是可以不用备案的。



**在netlify网站上添加自定义域名**

![image-20220604215655199](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604215655199.png)



**修改`DNS`，设置域名重定向**

因为是在腾讯云上购买的域名，所以登录https://www.dnspod.cn/后，在我的域名栏就可以看到已经购买的域名了。

![image-20220604220515109](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604220515109.png)

若是在其他平台购买的域名也可以直接添加到这里

![image-20220604220546957](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604220546957.png)

添加记录

![image-20220604220731566](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604220731566.png)



这里添加`Netlify`上设置的网站域名。我的是[shoufei.netlify.app](https://shoufei.netlify.app/)。默认的是字母加数字的，这里我自定义了的。添加好域名映射后就可以用自己的域名访问博客网站了。



**申请SSL证书**

点击右侧的`SSL`图标即可免费申请

![image-20220604220924594](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604220924594.png)

网站不安全的标志

![image-20220604221128103](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604221128103.png)

增加SSL证书后

![image-20220604221147424](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220604221147424.png)



参考：

[https://zhuanlan.zhihu.com/p/77651304](https://zhuanlan.zhihu.com/p/77651304)



## 提交网站给搜索引擎

[https://www.sousuoyinqingtijiao.com/](https://www.sousuoyinqingtijiao.com/)

在该网站提交自己的网站给各个搜索引擎。



## 参考



[https://uchuhimo.me/2017/04/11/genesis/](https://uchuhimo.me/2017/04/11/genesis/)

[http://ibruce.info/2013/11/22/hexo-your-blog/](http://ibruce.info/2013/11/22/hexo-your-blog/)

Hexo官方手册:

[https://hexo.io/docs/](https://hexo.io/docs/)



Hexo主题：

[https://hexo.io/themes/](https://hexo.io/themes/)



---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
