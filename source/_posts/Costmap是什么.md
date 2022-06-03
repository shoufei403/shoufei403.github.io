---
title: Costmap是什么？ #文章页面上的显示名称，一般是中文
date: 2022-05-02 15:30:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2,Costmap] #文章标签，可空，多标签请用格式，注意:后面有个空格
# description: 详细介绍了costmap
---

## costmap是什么？





![costmap](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200201191858626.png)

costmap翻译过来是代价地图的意思。由SLAM算法生成栅格地图。我们为栅格地图中的每一个栅格分配一个代价值，这样就形成了costmap。路径规划算法则可以在具有代价的栅格地图上生成路径。规划路径的生成则是强依赖于代价值。为了生成合适的路径，我们需要为每个栅格分配合适的代价值。最开始想到的是在单层的costmap中更新每个栅格的代价，然后直接给路径规划算法。但这样会引起诸多问题。比如因为所有的数据都在同一个costmap中更新，任何一个数据的变动都需要拿到之前其他的数据重新一起计算代价值。比如数据更新的地图范围也不好确定。比如当数据类型多了之后，数据整合的顺序不好控制。

后来想到将单层的costmap分成多层是个好办法。如上图所示，一层costmap只用同一种数据来更新。比如最底层的static map就是SLAM算法生成的静态地图。使用静态地图数据生成一层costmap。Obstacles 层则是由传感器数据更新的costmap层。甚至可以根据某些特殊目的自定义一个costmap层，使生成的路径规避某些区域。这在单层的costmap算法中是很难实现的。最后将所有的costmap层按特定的顺序组合起来形成了layered_costmap。可以看到这是一种更为灵活，扩展性也更强的方法。 
<!--more-->
## ROS中的costmap_2d功能包
注意这里是以ROS1中的costmap_2d功能包来介绍的。ROS1中的costmap_2d功能包和ROS2中的基本一致。算法流程架构没有太多变化。但是ROS2中的costmap_2d功能包增加了CostmapFilter模块，可以方便实现虚拟墙和速度限制区等功能。请查看这篇文章了解详情。  



costmap_2d功能包实现了上述的分层costmap。主要有static层，obstacle层，voxel_layer层和inflation层。每个层的数据类型是以插件的形式提供的。如下图所示：
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200202160112364.png)



其中inflation层使用InflationLayer类型数据，static层使用StaticLayer类型数据，而obstacle层可以选择VoxelLayer数据或者ObstacleLayer数据。插件的具体配置是在global_costmap_params.yaml文件和local_costmap_params.yaml文件中。

```bash
   plugins:
   - {name: static_layer,        type: "costmap_2d::StaticLayer"}
   - {name: obstacle_layer,      type: "costmap_2d::VoxelLayer"}
   - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}
```

各层数据类型的组成和继承关系如下图：


![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200202164903843.png)



Costmap2D类维护了每个栅格的代价值。Layer类是虚基类，它统一了各插件costmap层的接口。其中最主要的接口函数有：
**initialize函数**，它调用onInitialize函数，分别对各costmap层进行初始化；

**matchSize函数**，在StaticLayer类和ObstacleLayer类中，该函数调用了CostmapLayer类的matchSize函数，初始化各costmap层的size，分辨率，原点和默认代价值，并保持与layered_costmap一致。对于inflationLayer类，根据膨胀半径计算了随距离变化的cost表。后面就可以用距离来查询膨胀栅格的cost值。同时定义了seen_数组，该数组用于标记栅格是否已经被遍历过。对于VoxelLayer类，则初始化了体素方格的size；

**updateBounds函数**，调整当前costmap层需要更新的大小范围。对于StaticLayer类，确定costmap的更新范围为静态地图的大小（**注意：静态层一般只用在全局costmap中。**）。对于ObstacleLayer类，遍历clearing_observations中的传感器数据，确定障碍物的边界。并且根据marking observations中的传感器数据将障碍物位置处的栅格cost值设为LETHAL_OBSTACLE。对于inflationLayer类，在ObstacleLayer层的基础上增加膨胀半径（inflation_radius_）的范围。对于VoxelLayer类，大体与ObstacleLayer类一样，但考虑了z轴的数据。

**updateCosts函数**，将该层的cost值合并到主cospmap中，即LayeredCostmap包含的costmap层。

其中initialize函数和matchSize函数分别只执行一次。updateBounds函数和updateCosts函数则会周期执行，其执行频率由map_update_frequency决定。

CostmapLayer类同时继承了Layer类和Costmap2D类，并提供了几个更新cost值的操作方法。StaticLayer类和ObstacleLayer类需要保存实例化costmap层的cost值，所以都继承了CostmapLayer类。StaticLayer类使用静态栅格地图数据更新自己的costmap。ObstacleLayer类使用传感器数据更新自己的costmap。VoxelLayer类相对于ObstacleLayer类则多考虑了z轴的数据。效果的区别主要体现在障碍物的清除方面。一个是二维层面的清除，一个是三维里的清除。

算法的主要执行流程图如下：
![在这里插入图片描述](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/20200202230758394.png)

## 问题：

1.**每一层的实例都维护了一个costmap_变量吗？**  
static map和Obstacles层都有一个costmap_，而layered_costmap类也维护了一个costmap_，并且这个costmap_最终组合了其他几个层的costmap_。而inflation层没有维护costmap_，它直接将cost值更新到了LayeredCostmap的costmap_里。

2.**膨胀半径为什么还要另外设置而不是通过footprint自动计算得到？**  
算法中仅使用footprint来计算inscribed_radius_（底盘内径）和circumscribed_radius_（底盘外径）。膨胀半径使用inflation_radius参数来设定。

3.**footprint_padding参数有什么作用？ **   
使用footprint_padding_参数将底盘轮廓向x轴和y轴方向进行了膨胀。

4.**地图更新了会发生什么？ **   
地图更新后所有层的costmap尺寸都要更新。layered_costmap层会调用resizeMap函数，该函数再调用每层的matchSize函数更新各层的size和其他数据。

## 参考

参考论文：Layered Costmaps for Context-Sensitive Navigation  
[http://wiki.ros.org/costmap_2d/](http://wiki.ros.org/costmap_2d/)

在公众号《首飞》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。