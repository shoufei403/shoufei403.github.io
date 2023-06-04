
### 添加blogs
将md文件放置在`source/_posts`目录中。

在md文件中添加固定的头部格式，用于文章分类。

```
---
title: 下载TurtleBot4源码来学习
categories: ROS2
tags:
  - ROS2
  - TurtleBot4
abbrlink: ae5fa866
date: 2022-06-12 20:03:16
---

```

设置标题浏览页面显示的内容

```
<!--more-->
```

为每篇博客生成永久连接
将头部内容中的`abbrlink`去掉。然后再使用`hexo g`命令编译后可生成永久连接。

### 生成静态页面的步骤
清楚之前的生成内容
```bash
hexo clean
```

生成博客静态页面
```bash
hexo g
```

本地查看生成的页面效果
```bash
hexo s
```

