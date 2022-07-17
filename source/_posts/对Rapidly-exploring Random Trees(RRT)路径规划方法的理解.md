---
title: 对Rapidly-exploring Random Trees(RRT)路径规划方法的理解
categories: 路径规划
tags:
  - RRT
abbrlink: b0762db8
date: 2022-07-17 15:17:16
---


RRT与PRM一样，也是概率完备且不最优的。概率完备是指只要解存在就一定能在某一时刻找到。但解不一定是最优的。RRT与PRM相比，有一个优势就是，它在构建图的过程中就在寻找路径。
## RRT的主要算法流程



![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200313151632317.png)





这份基于matlab的代码很好的展示了RRT的算法流程:

[https://github.com/emreozanalkan/RRT](https://github.com/emreozanalkan/RRT)


<!--more-->

## RRT的优劣分析
**优势：**

 - 相对于PRM更具有目标导向，它在构建图的过程中就在寻找路径。
 - 无需对系统进行建模，无需对搜索区域进行几何划分，在搜索空间的覆盖率高，搜索的范围广，可以尽可能的探索未知区域。

**劣势：**

 - RRT得到的路径往往不是最优的。有比较大的优化空间。
 - RRT是在整个地图空间中随机采样点，这样全范围的采样往往效率不高。可以考虑启发式的采样，让采样的点更具有目标导向。
 - 在狭窄的通道里不易获得采样点。所以有时无法通过狭窄通道。

 





## 在RRT上的改进

### Bidirectional-RRT/RRT-Connect

**算法流程：**
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200313173213333.png)
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200313180844809.png)

![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200316204057100.gif)

关于双向RRT的介绍来源于：[https://www.cnblogs.com/21207-iHome/p/7210543.html](https://www.cnblogs.com/21207-iHome/p/7210543.html)
### RRT*
RRT* 完全继承了RRT的特性，并在其基础上扩展了两个新的特性。正是由于新特性的加入，RRT* 可以生成更优的路径。同时由于算法步骤的增加和collision check的增加，计算消耗也增加了。
下面是RRT* 的算法流程：
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200315202746584.png)
其实整个RRT* 算法可以分为三个大的部分。第一部分是继承至RRT的。这一部分和RRT是一样的，就是寻找到$z_{new}$和$z_{nearest}$。这一过程就是伪代码中的4，5，6步骤。
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200315205709225.png)

**near neighbor search **  
在这一步骤中，我们要为新得到的$z_{new}$找一个合适的父节点。思路是，以$z_{new}$为圆心，以固定的半径$r$画圆。落在该圆范围内的节点就是$z_{new}$潜在的父节点。我们将$z_{new}$与每个邻近的节点相连（如下图节点$a$、$b$），计算$z_{new}$通过哪个邻近节点到达$z_{init}$的cost最小。注意，这里的cost一般是指路径长度。
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/2020031521065792.png)
最终我们会发现，$z_{new}$通过$z_{nearest}$到达$z_{init}$路径最短。这时$z_{nearest}$就会设置成$z_{new}$的父节点。



**rewiring tree operations**  
到这一步时，$z_{new}$已经和$z_{nearest}$建立了连接，形成了一条新的边。这时我们要考虑a节点和b节点通过$z_{new}$到达$z_{init}$路径会短一点吗？从我画的图直观来看a节点和b节点原来的连接似乎使得路径更短。所以可以保留其原来的连接，不进行更改。
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200315211953582.png)
为了说明这一步骤的功能，我们做一个**假设**。我认为b节点通过$z_{new}$到达$z_{init}$路径会短一点，即红色路径比黑色路径短。这时会发生什么? a节点和b节点间的连接会断开，b节点会将自己的父节点改为$z_{new}$。

通过不断迭代上述步骤，RRT*会逐渐生成越来越短的路径。

<iframe 
    width="498" 
    height="510" 
    src="https://player.bilibili.com/player.html?aid=96583530"
    frameborder="0" 
    allowfullscreen>
</iframe>



[https://blog.csdn.net/ljq31446/article/details/78867011](https://blog.csdn.net/ljq31446/article/details/78867011)

[A Comparison of RRT, RRT* and RRT*-Smart Path Planning Algorithms ](http://paper.ijcsns.org/07_book/201610/20161004.pdf)
## Informed RRT*
在寻找到第一条可行的路径前，Informed RRT* 所做的工作与RRT* 是一样的。不同之处在于，Informed RRT* 使用初始路径的长度来画一个以起始点和终止点为焦点的椭圆。
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200316225658310.png)
随着迭代的进行，路径会越来越短。椭圆的面积也越来越小。因为采样点的区域被限定了，采样效率更高。收敛到最优路径花的时间也更少。
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200316230645500.png)

下图是RRT* 和Informed RRT* 的效果图。可以看到Informed RRT* 只在限定的椭圆内优化路径。当路径优化到相同的cost时，RRT* 比Informed RRT* 多花了8倍多的时间。
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200315220523137.png)



<iframe 
    width="498" 
    height="510" 
    src="https://player.bilibili.com/player.html?aid=96825237"
    frameborder="0" 
    allowfullscreen>
</iframe>




论文:
[Informed RRT*: Optimal sampling-based path planning focused via direct sampling of an admissible ell](https://arxiv.org/abs/1404.2334)







---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)
