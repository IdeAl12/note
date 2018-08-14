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
- aaa

###	节点

ROS节点可以使用ROS客户库与其他节点通信。节点可以发布或接受一个话题。节点也可以提供货使用某种服务。

###	客户端库

ROS客户端库允许使用不同编程语言编写的节点之间相互通信：

	rospy = python 客户端库
	roscpp = c++ 客户端库
	
###	roscore
roscore 是你在运行所有ROS程序前首先要运行的命令。
### rosnode
- rosnode显示当前运行的ROS节点信息。 
- rosnode list 指令列出活跃的节点。
- rosout用于收集和记录节点调试输出信息，所以总在运行。
- rosnode info 命令返回的是关于一个特点节点的信息。

###	rosrun 
rosrun允许使用包名直接运行一个包内的节点（不需要知道这个包的路径）

```
$ rosrun [package_name] [node_name]
```

ROS的一个强大特性就是可以通过命令行重新配置名称。

```
$ rosrun turtlesim turtlesim_node __name:=my_turtle
```

## ROS话题
简介：
