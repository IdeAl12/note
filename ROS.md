[TOC]
# ROS
## ROS节点
简介：主要介绍ROS图（graph）概念和roscore、rosnode和rosrun命令工具的使用。

###	图概念

- Nodes：节点，一个节点即为一个可执行文件，可以通过ROS与其他节点进行通信。

- Messags：消息，消息是一种ROS数据类型，用于订阅或发布到一个话题。

- Topic：话题，节点可以发布消息到话题，也可以订阅话题以接收消息。

- Master：节点管理器，ROS名称服务

- rosout：在ROS中相当于stdout/stderr

- roscore：主机+rosout+参数服务器

## 节点

ROS节点可以使用ROS客户库与其他节点通信。节点可以发布或接受一个话题。节点也可以提供货使用某种服务。

###	客户端库

ROS客户端库允许使用不同编程语言编写的节点之间相互通信：

	rospy = python 客户端库
	roscpp = c++ 客户端库

###	roscore
roscore 是你在运行所有ROS程序前首先要运行的命令。

当出现`roscore cannot run as another roscore/master is already running. `

解决方法：

```
killall -9 roscore
killall -9 rosmaster
```
### rosnode
- rosnode显示当前运行的ROS节点信息。 
- rosnode list 指令列出活跃的节点。
- rosout用于收集和记录节点调试输出信息，所以总在运行。
- rosnode info 命令返回的是关于一个特点节点的信息。


##	rosrun 
rosrun允许使用包名直接运行一个包内的节点（不需要知道这个包的路径）

```
$ rosrun [package_name] [node_name]
```

ROS的一个强大特性就是可以通过命令行重新配置名称。

```
$ rosrun turtlesim turtlesim_node __name:=my_turtle
```

## ROS话题
简介：介绍ROS话题（topics）以及如何使用rostopic和rqt_plot命令工具。

### ROS Topics
```
$roscore
$rosrun turtlesim turtlesim_node
#使用键盘控制turtle的运动
$rosrun turtlesim turtle_teleop_key
```
turtlesim\_node节点和turtle\_teleop\_key节点之间是通过一个ROS话题来互相通信的。

turtle\_teleop\_key 在一个话题上发布按键输入消息，turtlesim则订阅该话题以接受该消息。使用rqt\_graph来显示当前运行的节点和话题。

#### rqt_graph
rqt_graph能够创建一个显示当前系统运行情况的动态图形，是rqt程序包的一部分。
`$rosrun rqt_graph rqt_graph`

#### rostopic
rostopic工具可以获取有关ROS话题的信息。

rostopic的子命令：

`$rostopic -h`

```
rostopic bw     display bandwidth used by topic
rostopic echo   print messages to screen
rostopic hz     display publishing rate of topic
rostopic list   print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type
```

#### 使用rostopic echo
rostopic echo可以显示在某个话题上发布的数据。

用法：

`$ rostopic echo [topic]`

#### 使用rostopic list

rostopic list能够列出所有当前订阅和发布的话题。

```
$ rostopic list -h

Usage: rostopic list [/topic]

Options:
  -h, --help            show this help message and exit
  -b BAGFILE, --bag=BAGFILE
                        list topics in .bag file
  -v, --verbose         list full details about each topic
  -p                    list only publishers
  -s                    list only subscribers
```

在rostopic list中使用verbose选项，会显示出所发布和订阅的话题及其类型的详细信息。

### ROS Messages

话题之间的通信是通过在节点之间发送ROS消息实现的。对于发布器（turtle_teleop_key）和订阅器（turtulesim_node）之间的通信，发布器和订阅器之间必须发送和接受相同类型的消息。话题的类型是有发布在它上面的消息类型决定的。使用rostopic type命令可以查看发布在某个话题上的消息类型。

#### 使用rostopic type

rostopic type命令用于查看所发布话题的消息类型。

用法：

`$rostopic type [topic]`

#### 使用rostopic pub

rostopic pub可以把数据发布到当前某个正在广播的话题上。

用法：

`$rostopic pub [topic] [msg_type] [args]`

示例：发送消息给turtlesim，以2.0大小的线速度和1.8大小的角速度开始移动。

`$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'`

- rostopic pub 发布消息到某个给定的话题。
- -1 （单个破折号）使rostopic发布一条消息后马上退出。
- `/turtle1/cmd_vel` 这是消息所发布到的话题名称。
- `geometry_msgs/Twist` 这是所发布消息的类型。
- -- （双破折号）告诉命令选项解析器接下来的参数部分都不是命令选项。这在参数里面包含有破折号-（比如负号）时是必须要添加的。

#### 使用rostopic hz

rostopic hz命令可以用来查看数据发布的频率。

用法：

`$ rostopic hz [topic]`

#### 使用rqt_plot

rqt_plot命令可以实时显示一个发布到某个话题上的数据变化图形。

## ROS服务和参数

简介：介绍ROS服务和参数的知识，以及命令行工具rosservice 和rosparam的使用方法。

### ROS Services

服务（services）是节点之间通讯的另一种方式。服务允许节点发送请求（request）并获得一个响应（response）。

### 使用rosservice

rosservice可以使用ROS客户端/服务器框架提供的服务。

使用方法：

```
rosservice list		输出可用服务的信息
rosservice call		调用带参数的服务
rosservice type		输出服务类型
rosservice find		依据类型寻找服务
rosservice uri 		输出服务的ROSRPC uri
```

#### rosservice list

`$ rosservice list`

#### rosservice type

查看服务的类型。

`$ rosservice type [service]`

#### rosservice call

使用rosservice call命令调用服务。

`$ rosservice call [service] [args]`

### 使用rosparam

rosparam可以存储并操作ROS参数服务器（parameter server）上的数据。参数服务器能够存储整型、浮点、布尔、字符串、字典和列表等数据类型。rosparam使用YAML标记语言的语法。YAML的表述：1 是整型, 1.0 是浮点型, one是字符串, true是布尔, [1, 2, 3]是整型列表, {a: b, c: d}是字典

使用方法：

```
rosparam set            设置参数
rosparam get            获取参数
rosparam load           从文件读取参数
rosparam dump           向文件中写入参数
rosparam delete         删除参数
rosparam list           列出参数名
```

#### rosparam list

查看参数服务器上都有哪些参数。

`$ rosparam list`

#### rosparam set和rosparam get

```
$ rosparam set [param_name]
$ rosparam get [param_name]
```

例如修改turtlesim背景颜色：

```
$ rosparam set background_r 150
#调用清除服务使得修改后的参数生效：
$ rosservice call clear
```

显示参数服务器上的所有内容：

`$ rosparam get/`

#### rosparam dump 和rosparam load

```
$ rosparam dump [file_name]
$ rosparam load [file_name] [namespace]
```

将所有的参数写入params.yaml文件：

```
$ rosparam dump params.yaml
```

将yaml文件重新载入新的命名空间，如copy空间。

```
$ rosparam load params.yaml copy
$ rosparam get copy/background_b
```

## 使用rqt_console和roslaunch

简介：介绍如何使用rqt_console和rqt_logger_level进行调试，以及如何使用roslaunch同时运行多个节点。

### 使用rqt_console和rqt_logger_level

rqt_console属于ROS日志框架（logging framework）的一部分，用来显示节点的输出信息。

rqt_logger_level允许我们修改节点运行时输出信息的日至等级，包括DEBUG、WARN、INFO和ERROR。

