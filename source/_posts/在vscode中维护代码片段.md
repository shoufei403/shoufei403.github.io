---
title: 在Vscode中维护代码片段
categories: 实用技巧
tags:
  - vscode
abbrlink: efe8d202
date: 2022-05-02 15:30:16
---



### vscode中的snippet功能简介

vscode的语言插件已经内置了一些snippet。例如：C++的插件。当你在编辑器里输入main时，会提示你可以插入的snippet。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/11c828cc9aca8e7404eff9e2616bd06f.png)

但是也可以自己定义`snippet`，然后通过关键字来触发插入`snippet`。

vscode支持非常多编程语言的snippet。通过`ctrl+shift+p` ，打开命令面板。输入`snippet` ，就能看到`snippet`的配置选项。 

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/8f3fe95d793716056d5dee46cb5742c4.png)



点击`Configure User Snippets` 后就可以选择配置哪种语言的`snippet`了。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/614d74cfed10a7edb2ea9a6b7ab28c10.png)

点击语言名称可以生成相应语言的json文件。这个json文件就是用于维护snippet的。其中的cpp.json和python.json文件就是分别维护相应语言snippet的文件。snippet的内容可以按照语言类别来维护。在编辑器里插入时，vscode会检测当前编辑的是什么语言。当输入snippet的触发词时，只会触发对应语言的snippet。

<!--more-->

点击`global snippets file` 时会生成一个全局的`snippet`维护文件。并且其后缀是`.code-snippets` 。该文件中维护的`snippet`可以在任意文本编辑过程中被触发。



vscode 中使用snippet的官方文档：

[Snippets in Visual Studio Code](https://code.visualstudio.com/docs/editor/userdefinedsnippets)



### 编辑snippet的方法

#### 安装VS Code Snippet Generator插件

在market里搜索VS Code Snippet Generator并安装。



#### 编写snippet文件

snippet文件的格式如下所示:

```json
	// Place your snippets for cpp here. Each snippet is defined under a snippet name and has a prefix, body and 
	// description. The prefix is what is used to trigger the snippet and the body will be expanded and inserted. Possible variables are:
	// $1, $2 for tab stops, $0 for the final cursor position, and ${1:label}, ${2:another} for placeholders. Placeholders with the 
	// same ids are connected.
	// Example:
	"Print to console": {
		"prefix": "log",
		"body": [
			"console.log('$1');",
			"$2"
		],
		"description": "Log output to console"
	},
```

其中`Print to console` 是`snippet` 的名称。`prefix` 是触发`snippet`的关键词。`body` 就是`snippet` 的代码内容。每行代码都需要用双引号包裹起来。`description` 就是对`snippet` 的描述。

需要特别注意的是，$1和$2 是`tabstop`。当我们插入`snippet` 时，光标会自动处于$1的位置。这时可以输入相应的内容。输好后按`tab` 键，光标切换到$2处。`tabstop` 是可以赋默认值的。

```json
	"Print to console1": {
		"prefix": "log",
		"body": [
			"console.log('${1:/*test1 */');",
			"${2:/*test2*/}"
		],
		"description": "Log output to console"
	},
```



从上面看，编辑`snippet` 维护文件还挺麻烦的。找到一个插件（VS Code Snippet Generator）可以将普通代码转换成上面描述的格式。这里以cpp为例，描述一下操作步骤：

1. 先拷贝需要制作成`snippet`的代码片段到cpp.json文件中。
2. 在cpp.json文件中选中刚才拷贝过来的代码。
3. 使用`ctrl+shift+p` 快捷键打开命令面板。输入`snippet` 。然后点击`Generate snippet JSON block`。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/a5a03b7f7669115c1669ca0c0fe3e72c.png)

4. 注意加上`,`。完工。 



编辑好的`snippet` 文件存放在`~/.config/Code/User/snippets/` 目录中。

1.63版本的vscode已经支持使用github同步配置了。建议开启。因为该同步操作也可以把`snippet`文件同步到github。一处添加处处可用。

### 使用snippet功能

当键入`snippet` 关键词（即文件中prefix的值）时会自动提示可以插入的`snippet`。

![img](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/b49d932cf402ab0fde9e87a8413305ad.png)

提示的左侧会显示`snippet` 关键词（即prefix的值），右侧则显示`snippet` 的名称。如果键入关键词没有触发可选项，可以手动按`ctrl+space空格` 来触发。

