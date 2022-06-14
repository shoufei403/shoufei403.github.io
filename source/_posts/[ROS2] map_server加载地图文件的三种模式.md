---
title: ROS2 map_server加载地图文件的三种模式
categories: ROS2
tags:
  - ROS2
  - map_server
abbrlink: 9341ee9
date: 2022-05-26 15:30:16
---

## map的数据类型

`map`话题的类型是`nav_msgs::msg::OccupancyGrid`。使用下面的命令可以查询该类型的数据结构。  

```bash
ros2 interface show nav_msgs/msg/OccupancyGrid
```

`nav_msgs::msg::OccupancyGrid`的数据结构：

```bash
# This represents a 2-D grid map
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

# MetaData for the map
MapMetaData info
        builtin_interfaces/Time map_load_time
                int32 sec
                uint32 nanosec
        float32 resolution
        uint32 width
        uint32 height
        geometry_msgs/Pose origin
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1

# The map data, in row-major order, starting with (0,0).
# Cell (1, 0) will be listed second, representing the next cell in the x direction.
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently,
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown.
int8[] data
```

其中`data`数据成员用于存储地图中的每个栅格值。`nav_msgs::msg::OccupancyGrid`存储的栅格值范围在[0~100]。0表示栅格未被占用，100表示栅格被占用了，而0到100之间表示被占用的程度。-1表示未知区域。  

`info`成员变量中主要存储地图文件的一些参数。比如：地图大小，分辨率，原点等信息。

<!--more-->

## 加载map的三种模式

`map_server`功能包支持加载三种类型的图片文件：PGM/PNG/BMP。图片中每个像素的颜色亮度值将被转化成`nav_msgs::msg::OccupancyGrid`类型中的栅格值，存储在`data`成员变量中。

加载地图有下面三种方式：

- `trinary`
- `scale`
- `raw`

其中`trinary`为默认的加载方式。  

地图的加载方式通常会被配置在地图文件对应的配置文件中。该配置文件的内容如下：

```yaml
image: map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```



代码中对应的数据结构体：

```c++
struct LoadParameters
{
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};
```



其中`mode`如果不在`yaml`文件中写明的话就会使用默认的`trinary`。  `free_thresh`和`occupied_thresh`是判断栅格是否被占用的阈值。  



地图图片中每个像素可能有多个颜色通道。比如：`RGB`。像素的明暗程度值是通过求取各个颜色通道的明暗程度值得到的。像素的明暗程度值的范围在[0~1.0]。下面是代码的实现：

```c++
      auto pixel = img.pixelColor(x, y);

      std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(),
        pixel.blueQuantum()};
      if (load_parameters.mode == MapMode::Trinary && img.matte()) {
        // To preserve existing behavior, average in alpha with color channels in Trinary mode.
        // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
        channels.push_back(MaxRGB - pixel.alphaQuantum());
      }
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      /// on a scale from 0.0 to 1.0 how bright is the pixel?
      double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied. Otherwise, it's vice versa.
      /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
      double occ = (load_parameters.negate ? shade : 1.0 - shade);
```



默认情况下，我们认为颜色越暗，栅格被占用的概率越大。颜色越亮，栅格被占用的概率越小。  



但是当`negate`置为1时，这个逻辑就反过来了。颜色越暗，栅格被占用的概率越小。颜色越亮，栅格被占用的概率越大。 



### `trinary`模式下的栅格赋值方法

`trinary`模式下的判断比较简单。主要是像素的明暗程度值小于`free_thresh`就表示栅格没有被占用，给栅格赋0。若大于`occupied_thresh`就认为被占用了，给栅格赋100。在`free_thresh`和`occupied_thresh`之间就认为是状态不明，给栅格赋-1。下面是具体的代码实现：  

```C++
case MapMode::Trinary:
    if (load_parameters.occupied_thresh < occ) {
    map_cell = nav2_util::OCC_GRID_OCCUPIED;
    } else if (occ < load_parameters.free_thresh) {
    map_cell = nav2_util::OCC_GRID_FREE;
    } else {
    map_cell = nav2_util::OCC_GRID_UNKNOWN;
    }
```



### `scale`模式下的栅格赋值方法

当像素的`Alpha`值不为0，栅格值被设定为状态不明。像素的明暗程度值小于`free_thresh`就表示栅格没有被占用，给栅格赋0。若大于`occupied_thresh`就认为被占用了，给栅格赋100。在`free_thresh`和`occupied_thresh`之间就线性转换到[0~100]之间。

```c++
case MapMode::Scale:
    if (pixel.alphaQuantum() != OpaqueOpacity) {
        map_cell = nav2_util::OCC_GRID_UNKNOWN;
    } else if (load_parameters.occupied_thresh < occ) {
        map_cell = nav2_util::OCC_GRID_OCCUPIED;
    } else if (occ < load_parameters.free_thresh) {
        map_cell = nav2_util::OCC_GRID_FREE;
    } else {
        map_cell = std::rint(
            (occ - load_parameters.free_thresh) /
            (load_parameters.occupied_thresh - load_parameters.free_thresh) * 100.0);
    }
    break;
```





### `raw`模式下的栅格赋值方法

将像素的明暗程度值当成是百分比，通过下面代码描述的方式对栅格值赋值。

```c++
case MapMode::Raw: {
    double occ_percent = std::round(shade * 255);
    if (nav2_util::OCC_GRID_FREE <= occ_percent &&
    	occ_percent <= nav2_util::OCC_GRID_OCCUPIED)
    {
    	map_cell = static_cast<int8_t>(occ_percent);
    } else {
    	map_cell = nav2_util::OCC_GRID_UNKNOWN;
    }
    	break;
    
```



参考：

[https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html#prepare-filter-mask](https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html#prepare-filter-mask)



___

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。
