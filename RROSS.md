[TOC]

# Ros

## 简介

ROS(robot operating system)是用于机器人的一种开元的后操作系统，或说是此操作系统。

主要特点：

- 点对点设计
- 不依赖编程语言
- 精简与集成
- ROS-agnostic库
- 便与测试
- 规模
- 开源

## ROS总体框架及基本命令

### ROS总体框架

ROS系统有三级概念：文件系统级、计算图级、社区级。

#### 文件系统级

ROS文件系统级指的是可以再硬盘上查看的ROS源代码，包含如下几种形式：

- 功能包：组织软件的主要形式，可能包含ROS运行过程，一个ROS依赖库、数据库、配置文件或组织在一起的任何其他文件。
- Manifest：提供关于功能包的元数据（meta data），定义功能包之间的依赖关系。
- 功能包集
- Stack Manifest：指的是stack.xml文件，提供了关于功能包集的相关信息
- Message（msg）type：消息的描述，定义了ROS发送的消息的数据结构。
- Service（srv）type：服务的描述，定义了ROS中需求和响应的数据结构。

一个功能包就是带有一个manifest.xml文件的文件夹，一个功能包集就是一个带有stack.xml文件的文件夹。

#### 计算图级

计算图是ROS处理数据的一种点对点的网络形式。基本的概念：

1. 节点：执行计算的过程。
2. 节点管理器：为其他计算图提供了名称注册和查找的功能。
3. 参数服务器：允许数据通过在一个中心位置的关键词来存储。
4. 消息：节点之间通过消息来通信。一个消息是一个有类型域构成的简单的数据结构。
5. 主题：消息通过一个带有发布和订阅功能的传输系统来传送。一个节点通过报消息发送到一个给定的主题来发布一个消息。主题是用于识别消息内容的名称。
6. 服务：服务被定义为一对消息结构：一个用于请求，一个用于回复。
7. 消息记录包：一种用于存放和回放ROS消息数据的格式。

#### 社区级

ROS网络上进行代码发布的一种表现形式。每个研究所和组织会以软件仓库为单位来发布他们的代码。

#### 更高层概念

1. 坐标系/坐标变换：tf功能包提供了一个机遇ROS的分布式框架，可以随着时间的推移计算多个坐标系的位置。
2. 行动/任务：actionlib功能包定义一个通用的机遇主题的界面，来处理ROS中的抢占任务。
3. 消息本体。
4. 插件：Plugins提供一个库，可以再C++代码中动态加载库。
5. 滤波器：C++库，用于数据处理。
6. 机器人模型：URDF功能包定义了一个XML格式来描述一个机器人模型，并提供了一个C++解析器。

#### 名称

##### 计算图源名称

计算图源名称提供了封装作用。没一个源被定义在一个命名空间内，该源可以与其他源共享资源。这种封装分离了系统的不同部分，避免因为偶然的命名错误或者全局的名称劫持而导致错误。

###### 名称解析

ROS有四种类型的计算图源名称：基本名称、相对名称、全局名称和私有名称：

- base
- relative/name
- /global/name
- ~private/name

##### 功能包名称

在源名称前面加上所在源的功能包的名字。一些ROS相关的功能包源名称文件名包括：

- 消息（msg）类型
- 服务（srv）类型
- 节点类型

### ROS基本命令

#### 文件系统命令

1. rospack：rospack = ros + pack(age)

rospack是用于提取文件系统上的功能包信息的命令工具。

用法：

```
rospack <command> [options] [package]
```

2. rosstack: rosstack = ros + stack

rosstack是用于提取文件系统上的功能包集信息的命令工具，可以知晓一系列与功能包集相关的命令，如给出功能包集列表、功能包集的依赖项列表等。

用法：

```
rosstack [options] <command> [stack]
```

3. roscd: roscd = ros + cd

改变路径到相应的功能包或功能包集。roscd仅仅列出在ROS_PACKAGE_PATH目录下的功能包。

用法：

```
roscd [package[/subdir]]
```

4. rosls: rosls = ros + ls

罗列相应的功能包、功能包集文件夹的命令。是rosbash套件的一部分。可以通过名称来列表一个文件夹下的文件，而不是根据目录列表。

用法：

```
rosls [package[/subdir]]
```

5. roscreate-pkg

创建一个新的ROS功能包

用法：

```
roscreate-pkg [package_name]
roscreate-pkg [package_name] [depend1] [depend2] [depend3]
```

6. roscreate-stack

创建一个新的ROS功能包集，用法类似于5

7. rosdep

安装ROS功能包系统依赖文件

8. rosmake

编译安装一个ROS功能包

用法：

```
rosmake [package] :编译单个功能包
rosmake [package1] [package2] [package3]
```

9. roswtf

显示ROS系统或启动文件的错误或警告信息。

#### ROS核心命令

1. roscore: = ros + core

运行机遇ROS系统必须的节点和程序的集合。为了保证节点能够通信，至少要有一个roscore在运行。roscore当前定义为：

- master
- parameter server
- rosout

2. rosmsg/rossrv

显示消息或者服务的数据结构定义

- rosmsg show: 显示在消息中域的定义
- rosmsg users: 显示使用指定消息的代码
- rosmsg md5: 显示消息的md5值
- rosmsg package: 列出制度功能包中的所有消息
- rosnode packages: 列出带有该消息的所有功能包

3. rosrun

rosrun允许用户不必先改变到相应目录就可以执行在任意一个功能包下的可执行文件。

用法：

```
rosrun package executable
e.g. rosrun turlesim turlesim_node
```

4. rosnode

显示关于ROS节点（包括发布、订阅和连接）的调试信息。

- rosndoe ping：测试到一个节点的可连接性
- rosnode list：列出活动节点
- rosnode info：打印节点的信息
- rosnode machine：列出在特定机器上正在运行的节点
- rosnode kill：结束一个正在运行的节点  -a

5. roslaunch

通过SSH和在参数服务器上设置参数来局部和远程启动ROS节点。通过调用一个或多个XML配额制文件来完成启动过程。在配置文件中会对每一个要启动的节点进行描述。用法：

- 在不同的借口启动：roslaunch -p 1234 package filename.launch
- 在功能包内启动文件：roslaunch package filename.launch
- 在局部节点启动文件：roslaunch --local package filename.launch

6. rostopic

用于显示ROS主题（包括发布、订阅、发布频率和消息）调试信息的工具。

```
rostopic bw：显示主题的带宽
rostopic echo [topic]：输出主题信息到屏幕
rostopic hz [topic]：显示主题发布频率
rostopic list [/topic]：打印活动主题的信息
rostopic pub [topic] [msg_type] [args]：发布数据到主题
rostopic type [topic]：打印主题类型
rostopic find：通过类型查找主题
```

7. rosparam

一个获取和设置参数服务器上用YAML编码的文件工具。

```
rosparam set [param_name] 设置一个参数
rosparam get [param_name] 获取一个参数
rosparam load [file_name] [namespace] 从一个文件中调取一个参数
rosparam dump [file] 写参数到一个文件
rosparam delete 删除一个参数
rosparam list 列出参数名称
```

8. rosservice

用于列表和查询ROS服务器的工具。服务是节点之间相互通信的另一种方式，服务允许节点发送请求和接受响应。

```
rosservice list 打印活动服务的信息
rosservice node 打印提供一个服务的节点的名称
rosservice call [service] [args] 启动给定变量的服务
rosservice args 列出一个服务的变量
rosservice type [service] 打印服务类型
rosservice uri 打印服务ROSRPC uri
rosservice find 通过服务类型查找服务
```

### 工具

#### 3D可视化工具：rviz

rviz是ROS中的一个3D可视化工具。

命令参数及用法：

- -h, --help 
- -d, --display-config <arg> 开始时调用配置文件
- -t, --target-grame <arg> 设置目标坐标系为<arg>。覆盖在配置文件中指定的目标坐标系。
- -f, --fixed-frame <arg> 设置固定坐标系为<arg>。覆盖在配置文件中指定的目标坐标系。

启动：

```
rosrun rviz rviz
```

![屏幕快照 2018-09-16 下午7.00.28](/Users/lixiang/Desktop/屏幕快照 2018-09-16 下午7.00.28.png)