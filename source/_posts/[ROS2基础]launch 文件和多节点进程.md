---
title: launch 文件和多节点进程 #文章页面上的显示名称，一般是中文
date: 2022-05-29 10:31:16 #文章生成时间，一般不改，当然也可以任意修改
categories: ROS2 #分类
tags: [ROS2] #文章标签，可空，多标签请用格式，注意:后面有个空格
description: ROS2基础之launch文件和多节点进程
---
## launch 文件

`launch`文件可以同时配置和启动多个`ros`节点。`ROS2`中的`launch`文件可以用`Python`、`xml`、`yaml`来写。  



但`ROS2`中的`Python launch`文件更为灵活，功能也更加强大。可以用它执行一些其他的任务（比如**新建目录**，配置**环境变量**）。所以官方**推荐**的是使用`python`来写。而`launch`文件一般会放在功能包中的`launch`文件夹下面。如果想感受一下各种方式写`launch`文件的**效果**，可以点开下面的链接体会一下。

[https://docs.ros.org/en/galactic/How-To-Guides/Launch-file-different-formats.html](https://docs.ros.org/en/galactic/How-To-Guides/Launch-file-different-formats.html)



`launch`文件一般通过下面的命令启动：

```HTML
ros2 launch <package_name> <launch_file_name>
```

**值的注意的是**，当`package`编译时加了`--symlink-install`选项，在包内修改了`launch`文件，不用编译也是生效的。  



首先，我们来体验一下`launch`文件的功能。这里重声一下`launch`文件的作用：**配置**节点和**启动**节点。  

运行下面的命令，可以看到我们同时启动了两个小乌龟的窗口。  

```Bash
ros2 launch turtlesim multisim.launch.py
```

回想一下，我们之前启动小乌龟窗口时是用下面的命令启动的。

```Bash
ros2 run turtlesim turtlesim_node
```



那`launch`文件又是怎么同时启动两个小乌龟窗口的呢？我们看一下`launch`文件的内容。

存储路径：`/opt/ros/galactic/share/turtlesim/launch`

```Python
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
    ])
```

`launch`文件中定义了一个`generate_launch_description()` 函数，它返回`LaunchDescription` 。而在`LaunchDescription` 中写了需要启动的节点，当然还可以加其他的内容，比如参数声明等等。

运行`rqt_graph`，可以看到下面的节点图

![image-20220529092028148](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20220529092028148.png)



**可以发现**，launch文件启动了两次同一个执行文件。这个执行文件是`turtlesim`包中的`turtlesim_node`。可以在`/opt/ros/galactic/lib/turtlesim`目录下找到`turtlesim_node` 。`namespace`是为了让节点处于不同的命名空间内，这一点在上面的图中有体现出来。`output='screen'` 的意思是执行的节点打印的log直接在窗口中输出。除此之外，log也会存在`~/.ros/log` 目录下（这是默认路径，其实也是可以配置的）。



虽然`python launch`文件写起来比`xml`、`yaml`都要冗余，但也是有套路的。

可将`launch`文件分为四部分，这样比较好理解。

### 导入模块

这部分其实是`python`的语法。意思是，如果你想用另一个`python`文件的函数，需要将其`import`进来，类似于`C/C++`中的`include`语法。比如下面的写法：

```Python
from launch import LaunchDescription
import launch_ros.actions
```

### 参数声明

这一部分主要是声明有哪些参数，并且给每个参数赋上默认值。

**声明参数**

这一部分是在launch系统中，以名称来访问参数的值。意味着，上级`launch`文件可以传值给下级`launch`文件。

> These `LaunchConfiguration` substitutions allow us to acquire the value of the launch argument in any part of the launch description.

```Python
# Input parameters declaration
namespace = LaunchConfiguration('namespace')
params_file = LaunchConfiguration('params_file')
use_sim_time = LaunchConfiguration('use_sim_time')
#如果launch系统中没有这个参数，可以给它赋默认值
use_sim_time = LaunchConfiguration('use_sim_time', default='false')
autostart = LaunchConfiguration('autostart')
```

**给参数赋默认值**

这些参数是上级的launch文件传给下级的launch文件的参数。

> `DeclareLaunchArgument` is used to define the launch argument that can be passed from the above launch file or from the console.

```Python
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')
```

参数配置好后就可以输入到node节点中了。同时，这些参数也可以在启动launch文件的同时临时修改。

形式为下面的方式：

```Bash
ros2 launch <package_name> <launch_file_name> use_sim_time:=0
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

### 构建需启动的节点

这个部分主要是写明需要启动哪些节点。并且将节点需要的参数配置好。

```Python
 # Nodes launching commands

    start_map_saver_server_cmd = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            parameters=[configured_params])

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
                        
    start_rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
            
    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
```

节点中有8个参数。如果不需要配置可以不写，让它保持默认值。

前面已经提到`namespace`是为了让节点处于不同的命名空间内，这一点可以在上面的图中体现出来。`output='screen'` 的意思是执行的节点打印的log直接在窗口中输出。除此之外，log也会存在`~/.ros/log` 目录下（这是默认路径，其实也是可以配置的）。

补充一下其他几个参数

`package='nav2_lifecycle_manager'` 是指这个节点在哪个`ros2`包里。`executable='lifecycle_manager'` 指明需要运行的程序是`ros2`包中的哪一个执行文件（一个`ros`包是可以包含多个执行文件的）。

`parameters` 中可以配置`node`的参数。这些参数通常会在参数声明部分赋好值，这里直接传给节点即可。同时这里也可以直接传入`yaml`**参数文件**。`arguments=['-d', rviz_config_dir]` 是节点内部实现的参数。通常通过`main`函数的行参获取。比如下面这个命令：

```bash
run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
```

其中`-d`就是通过`main`函数传入进去的参数。



下面的参数用于将两个话题名称对等。**左边**是节点内部代码内写明的，**右边**为系统中其他节点提供的话题。

```Python
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
```

给节点配置参数文件的示例

```Python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('launch_tutorial'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         namespace='turtlesim2',
         name='sim',
         parameters=[config]
      )
   ])
```

注意`parameters=[config]`就是给节点配置`yaml`参数文件。



### 启动命令

这一部分只需将前面配置好的命令，用`add_action`方法加入即可。当然，这里其实有**两种写法**。一种是下面这种。

```Bash
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
```

另一种是直接将启动的内容写在`return`语句中。

```SQL
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
```





### launch文件中常用到的语句

**获取路径**

```CoffeeScript
import os
from ament_index_python.packages import get_package_share_directory

# Get the launch directory
bringup_dir = get_package_share_directory('nav2_bringup')#获取包的路径
launch_dir = os.path.join(bringup_dir, 'launch')#组合包内路径
```

**设置环境变量**

```CoffeeScript
from launch.actions import SetEnvironmentVariable

stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
        
```

**获取环境变量的值**

```Nginx
import os
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
```

**包含另外一个python launch文件**

```CoffeeScript
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
    launch_arguments={
        'map': map_dir,
        'use_sim_time': use_sim_time,
        'params_file': param_dir}.items(), #这里给内嵌的python launch文件传入参数
),
```

**给一组节点添加同一个命名空间**

```CoffeeScript
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

   ...
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   turtlesim_world_2_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('turtlesim2'),
         turtlesim_world_2,
      ]
   )
```

**在launch文件中执行命令**

```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```



### 分析几个launch文件

包含多个其他launch文件

```Python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   turtlesim_world_1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_1.launch.py'])
      )
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   broadcaster_listener_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/broadcaster_listener.launch.py']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )
   mimic_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/mimic.launch.py'])
      )
   fixed_frame_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/fixed_broadcaster.launch.py'])
      )
   rviz_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_rviz.launch.py'])
      )

   return LaunchDescription([
      turtlesim_world_1,
      turtlesim_world_2,
      broadcaster_listener_nodes,
      mimic_node,
      fixed_frame_node,
      rviz_node
   ])
# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    background_r_launch_arg = DeclareLaunchArgument(
        "background_r", default_value=TextSubstitution(text="0")
    )
    background_g_launch_arg = DeclareLaunchArgument(
        "background_g", default_value=TextSubstitution(text="255")
    )
    background_b_launch_arg = DeclareLaunchArgument(
        "background_b", default_value=TextSubstitution(text="0")
    )
    chatter_ns_launch_arg = DeclareLaunchArgument(
        "chatter_ns", default_value=TextSubstitution(text="my/chatter/ns")
    )

    # include another launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('demo_nodes_cpp'),
                'launch/topics/talker_listener.launch.py'))
    )
    # include another launch file in the chatter_ns namespace
    launch_include_with_namespace = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('chatter_ns')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('demo_nodes_cpp'),
                        'launch/topics/talker_listener.launch.py'))
            ),
        ]
    )

    # start a turtlesim_node in the turtlesim1 namespace
    turtlesim_node = Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    turtlesim_node_with_parameters = Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim',
            parameters=[{
                "background_r": LaunchConfiguration('background_r'),
                "background_g": LaunchConfiguration('background_g'),
                "background_b": LaunchConfiguration('background_b'),
            }]
        )

    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )

    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        chatter_ns_launch_arg,
        launch_include,
        launch_include_with_namespace,
        turtlesim_node,
        turtlesim_node_with_parameters,
        forward_turtlesim_commands_to_second_turtlesim_node,
    ])
```

临时添加`parameters`的方法

```python
parameters=[{
    "background_r": LaunchConfiguration('background_r'),
    "background_g": LaunchConfiguration('background_g'),
    "background_b": LaunchConfiguration('background_b'),
}]

parameters=[{'use_sim_time': use_sim_time},
    {'autostart': autostart},
    {'node_names': lifecycle_nodes}
])
```



`turtlebot3`启动导航的`launch`文件

存放路径：`turtlebot3/turtlebot3_navigation2/launch/navigation2.launch.py`

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
```

`turtlebot3`启动建图

存放路径：`turtlebot3/turtlebot3_cartographer/launch/cartographer.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
```

`Navigation2`启动定位程序

路径：`navigation2/nav2_bringup/bringup/launch/localization_launch.py`

```Python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['map_server', 'amcl']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])
```



参考：

[6.1 ROS2接点管理之launch文件](https://fishros.com/d2lros2foxy/#/chapt6/6.1ROS2接点管理之launch文件)

[Creating a launch file ‒ ROS 2 Documentation: Galactic documentation](https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Creating-Launch-Files.html)

[Launching/monitoring multiple nodes with Launch ‒ ROS 2 Documentation: Galactic documentation](https://docs.ros.org/en/galactic/Tutorials/Launch-system.html)

[Using ROS 2 launch for large projects ‒ ROS 2 Documentation: Galactic documentation](https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Using-ROS2-Launch-For-Large-Projects.html#build-and-run)

[Using substitutions in launch files ‒ ROS 2 Documentation: Galactic documentation](https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Using-Substitutions.html)

[Using event handlers in launch files ‒ ROS 2 Documentation: Galactic documentation](https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Using-Event-Handlers.html)



## 多节点进程

示例代码仓库：

```bash
git clone https://gitee.com/shoufei/ros2_galactic_turorials.git
```



### 在同一个进程中运行多个节点，节点间的通信实现零拷贝。

下面的命令可以演示这个过程

```bash
ros2 run intra_process_demo two_node_pipeline
```

**示例源码**可以在这里查看`ros2_galactic_turorials/demos/intra_process_demo/src/two_node_pipeline`

```c++

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

// Node that produces messages.
struct Producer : public rclcpp::Node
{
  Producer(const std::string & name, const std::string & output)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub]() -> void {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        static int32_t count = 0;
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = count++;
        printf(
          "Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
          reinterpret_cast<std::uintptr_t>(msg.get()));
        pub_ptr->publish(std::move(msg));
      };
    timer_ = this->create_wall_timer(1s, callback);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Node that consumes messages.
struct Consumer : public rclcpp::Node
{
  Consumer(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input,
      10,
      [](std_msgs::msg::Int32::UniquePtr msg) {
        printf(
          " Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
          reinterpret_cast<std::uintptr_t>(msg.get()));
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("producer", "number");
  auto consumer = std::make_shared<Consumer>("consumer", "number");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
```



### 一个进程多个线程分别运行不同节点

```Dockerfile
ros2 run examples_rclcpp_multithreaded_executor multithreaded_executor
```

示例源码可以在这里查看`/ros2_galactic_turorials/examples/rclcpp/executors/multithreaded_executor`

```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * A small convenience function for converting a thread ID to a string
 **/
std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

/* For this example, we will be creating a publishing node like the one in minimal_publisher.
 * We will have a single subscriber node running 2 threads. Each thread loops at different speeds, and
 * just repeats what it sees from the publisher to the screen.
 */

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
  : Node("PublisherNode"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello World! " + std::to_string(this->count_++);

        // Extract current thread
        auto curr_thread = string_thread_id();

        // Prep display message
        RCLCPP_INFO(
          this->get_logger(), "\n<<THREAD %s>> Publishing '%s'",
          curr_thread.c_str(), message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

class DualThreadedNode : public rclcpp::Node
{
public:
  DualThreadedNode()
  : Node("DualThreadedNode")
  {
    /* These define the callback groups
     * They don't really do much on their own, but they have to exist in order to
     * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
     */
    callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_subscriber2_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_subscriber2_;

    subscription1_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      rclcpp::QoS(10),
      // std::bind is sort of C++'s way of passing a function
      // If you're used to function-passing, skip these comments
      std::bind(
        &DualThreadedNode::subscriber1_cb,  // First parameter is a reference to the function
        this,                               // What the function should be bound to
        std::placeholders::_1),             // At this point we're not positive of all the
                                            // parameters being passed
                                            // So we just put a generic placeholder
                                            // into the binder
                                            // (since we know we need ONE parameter)
      sub1_opt);                  // This is where we set the callback group.
                                  // This subscription will run with callback group subscriber1

    subscription2_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      rclcpp::QoS(10),
      std::bind(
        &DualThreadedNode::subscriber2_cb,
        this,
        std::placeholders::_1),
      sub2_opt);
  }

private:
  /**
   * Simple function for generating a timestamp
   * Used for somewhat ineffectually demonstrating that the multithreading doesn't cripple performace
   */
  std::string timing_string()
  {
    rclcpp::Time time = this->now();
    return std::to_string(time.nanoseconds());
  }

  /**
   * Every time the Publisher publishes something, all subscribers to the topic get poked
   * This function gets called when Subscriber1 is poked (due to the std::bind we used when defining it)
   */
  void subscriber1_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    auto message_received_at = timing_string();

    // Extract current thread
    RCLCPP_INFO(
      this->get_logger(), "THREAD %s => Heard '%s' at %s",
      string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
  }

  /**
   * This function gets called when Subscriber2 is poked
   * Since it's running on a separate thread than Subscriber 1, it will run at (more-or-less) the same time!
   */
  void subscriber2_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    auto message_received_at = timing_string();

    // Prep display message
    RCLCPP_INFO(
      this->get_logger(), "THREAD %s => Heard '%s' at %s",
      string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // You MUST use the MultiThreadedExecutor to use, well, multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  auto pubnode = std::make_shared<PublisherNode>();
  auto subnode = std::make_shared<DualThreadedNode>();  // This contains BOTH subscriber callbacks.
                                                        // They will still run on different threads
                                                        // One Node. Two callbacks. Two Threads
  executor.add_node(pubnode);
  executor.add_node(subnode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```



### 一个进程多个线程，每个线程运行不同节点，并设置每个线程的优先级和占用哪个CPU

```Dockerfile
ros2 run examples_rclcpp_cbg_executor ping_pong
```

示例源码可以在这里查看`/ros2_galactic_turorials/examples/rclcpp/executors/cbg_executor/src`

```c++
#include <cinttypes>
#include <cstdlib>
#include <ctime>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "examples_rclcpp_cbg_executor/ping_node.hpp"
#include "examples_rclcpp_cbg_executor/pong_node.hpp"
#include "examples_rclcpp_cbg_executor/utilities.hpp"

using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using namespace std::chrono_literals;

using examples_rclcpp_cbg_executor::PingNode;
using examples_rclcpp_cbg_executor::PongNode;
using examples_rclcpp_cbg_executor::configure_thread;
using examples_rclcpp_cbg_executor::get_thread_time;
using examples_rclcpp_cbg_executor::ThreadPriority;

/// The main function composes a Ping node and a Pong node in one OS process
/// and runs the experiment. See README.md for an architecture diagram.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create two executors within this process.
  rclcpp::executors::SingleThreadedExecutor high_prio_executor;
  rclcpp::executors::SingleThreadedExecutor low_prio_executor;

  // Create Ping node instance and add it to high-prio executor.
  auto ping_node = std::make_shared<PingNode>();
  high_prio_executor.add_node(ping_node);

  // Create Pong node instance and add it the one of its callback groups
  // to the high-prio executor and the other to the low-prio executor.
  auto pong_node = std::make_shared<PongNode>();
  high_prio_executor.add_callback_group(
    pong_node->get_high_prio_callback_group(), pong_node->get_node_base_interface());
  low_prio_executor.add_callback_group(
    pong_node->get_low_prio_callback_group(), pong_node->get_node_base_interface());

  rclcpp::Logger logger = pong_node->get_logger();

  // Create a thread for each of the two executors ...
  auto high_prio_thread = std::thread(
    [&]() {
      high_prio_executor.spin();
    });
  auto low_prio_thread = std::thread(
    [&]() {
      low_prio_executor.spin();
    });

  // ... and configure them accordinly as high and low prio and pin them to the
  // first CPU. Hence, the two executors compete about this computational resource.
  const int CPU_ZERO = 0;
  bool ret = configure_thread(high_prio_thread, ThreadPriority::HIGH, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure high priority thread, are you root?");
  }
  ret = configure_thread(low_prio_thread, ThreadPriority::LOW, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure low priority thread, are you root?");
  }

  // Creating the threads immediately started them.
  // Therefore, get start CPU time of each thread now.
  nanoseconds high_prio_thread_begin = get_thread_time(high_prio_thread);
  nanoseconds low_prio_thread_begin = get_thread_time(low_prio_thread);

  const std::chrono::seconds EXPERIMENT_DURATION = 10s;
  RCLCPP_INFO_STREAM(
    logger, "Running experiment from now on for " << EXPERIMENT_DURATION.count() << " seconds ...");
  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // Get end CPU time of each thread ...
  nanoseconds high_prio_thread_end = get_thread_time(high_prio_thread);
  nanoseconds low_prio_thread_end = get_thread_time(low_prio_thread);

  // ... and stop the experiment.
  rclcpp::shutdown();
  high_prio_thread.join();
  low_prio_thread.join();

  ping_node->print_statistics(EXPERIMENT_DURATION);

  // Print CPU times.
  int64_t high_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    high_prio_thread_end - high_prio_thread_begin).count();
  int64_t low_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    low_prio_thread_end - low_prio_thread_begin).count();
  RCLCPP_INFO(
    logger, "High priority executor thread ran for %" PRId64 "ms.", high_prio_thread_duration_ms);
  RCLCPP_INFO(
    logger, "Low priority executor thread ran for %" PRId64 "ms.", low_prio_thread_duration_ms);

  return 0;
}
```



[https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html](https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html)

---

**觉得有用就点赞吧！**

我是首飞，一个帮大家**填坑**的机器人开发攻城狮。

另外在公众号《**首飞**》内回复“机器人”获取精心推荐的C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)

