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

```
$ rosrun rqt_console rqt_console
$ rosrun rqt_logger_level rqt_logger_level
```

#### 日志等级

日志等级按以下优先顺序排列：

```
Fatal
Error
Warn
Info
Debug
```

通过设置日至等级可以获取该等级及其以上优先等级的所有日志消息。比如，将日志等级设为Warn时，会得到 Warn、Error和 Fatal 这三个等级的所有日志消息。

#### 使用roslaunch

roslaunch可以用来启动定义在launch文件中的多个节点。

用法：

```
$ roslaunch [package] [filename.launch]
#创建一个launch文件夹
$ mkdir launch
$ cd launch
```

#### Launch文件及解析

创建一个turtlemimic.launch的launch文件。 

```xml
 <launch>

      <group ns="turtlesim1">
        <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>

      <group ns="turtlesim2">
        <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>

      <node pkg="turtlesim" name="mimic" type="mimic">
        <remap from="input" to="turtlesim1/turtle1"/>
        <remap from="output" to="turtlesim2/turtle1"/>
      </node>

    </launch>
```

创建了两个节点分组并以命名空间（namespace）标签来区分。

```xml
   <node pkg="turtlesim" name="mimic" type="mimic">
       <remap from="input" to="turtlesim1/turtle1"/>
       <remap from="output" to="turtlesim2/turtle1"/>
     </node>
```

启动模仿节点，并将所有话题的输入和输出分别重命名为turtlesim1和turtlesim2，这样就会使turtlesim2模仿turtlesim1。

- launch: Root element of the launch file
- node: Each <node> tag specifies a node to be launched
- name: Name of the node (free to choose)
- pkg: Package containing the node
- type: Type of the node, there must be a corresponding executable with the same name
- output: Specifies where to output log messages
- 注意自关闭标记的语法差异：<tag></tag> and <tag/>

## 使用rosed编辑ROS中的文件

简介：介绍如何使用rosed简化编辑过程。

### 使用rosed

rosed是rosbash的一部分。利用它可以直接通过package名来获取到待编辑的文件而无需指定该文件的存储路径。

用法：

`$ rosed [package_name] [filename]`

例：

`$ rosed roscpp Logger.msg`

## 创建ROS消息和ROS服务

简介：介绍如何创建并编译ROS消息的服务，以及rosmsg，rossrv和roscp命令行工具的使用。

### 消息msg和服务srv介绍

- 消息msg：msg文件是一个描述ROS中所使用消息类型的简单文本。会被用来生成不同语言的源代码。msg文件存放在package的msg目录下。msg文件实际上就是每行声明一个数据类型和变量名。可以使用的数据类型如下：

- ```
  int8, int16, int32, int64 (plus uint*)
  float32, float64
  string
  time, duration
  other msg files
  variable-length array[] and fixed-length array[C]
  ```

  

- 服务srv：一个srv文件描述一项服务，包含两个部分：请求和响应。srv文件则存放在srv目录下。

ROS中一个特殊的数据类型：Header。它含有时间戳和坐标信息。在msg文件的第一行经常可以看到Header header的声明。

srv文件分为请求和响应两部分，由---分隔。

### 使用msg

#### 创建一个msg

```
$ cd ~/catkin_ws/src/beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```

以确保msg文件被转化为C++、Python和其他语言的源代码，package.xml文件中需要包含以下语句：

```xml
  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
```

#### 使用rosmsg

通过rosmsg show命令，检查ROS是否能够识别消息。

用法：

`$ rosmsg show [message type]`

例：

`$ rosmsg show beginner_tutorials/Num`

### 使用srv

##### 创建一个srv

```
$ roscd beginner_tutorials
$ mkdir srv
```

roscp可以实现将文件从一个package复制到另一个package的功能。

用法：

```
$ roscp [package_name] [file_to_copy_path] [copy_path]
```

#### 使用rossrv

通过rossrv show命令，检查ROS是否能够识别该服务。

用法：

`$ rossrv show <service type>`

### 获得帮助

```
$ rosmsg -h
Commands:
  rosmsg show Show message description
  rosmsg users  Find files that use message
  rosmsg md5  Display message md5sum
  rosmsg package  List messages in a package
  rosmsg packages List packages that contain messages
  
$ rosmsg show -h
Usage: rosmsg show [options] <message type>

Options:
  -h, --help  show this help message and exit
  -r, --raw   show raw message text, including comments
```

## 编写简单的消息发布器和订阅器（c++）

### 编写发布器节点

节点Node是ROS网络中的可执行文件。创建一个发布器节点talker，将不断在ROS网络中广播消息。

#### 源代码

在beginner_tutorials package 路径下创建一个src文件夹，用来放置beginner_tutorials package 的所有源代码。

```
$ mkdir -p ~/catkin_ws/src/beginner_tutorials/src
```

```C++
   #include "ros/ros.h"
   #include "std_msgs/String.h"

   #include <sstream>

   /**
   * This tutorial demonstrates simple sending of messages over the ROS system.
    */
   int main(int argc, char **argv)
   {
     /**
     * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node.
      *
      * You must call one of the versions of ros::init() before using any other
      * part of the ROS system.
      */
     ros::init(argc, argv, "talker");

     /**
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
      */
     ros::NodeHandle n;

     /**
      * The advertise() function is how you tell ROS that you want to
      * publish on a given topic name. This invokes a call to the ROS
      * master node, which keeps a registry of who is publishing and who
      * is subscribing. After this advertise() call is made, the master
      * node will notify anyone who is trying to subscribe to this topic name,
      * and they will in turn negotiate a peer-to-peer connection with this
      * node.  advertise() returns a Publisher object which allows you to
      * publish messages on that topic through a call to publish().  Once
      * all copies of the returned Publisher object are destroyed, the topic
      * will be automatically unadvertised.
      *
      * The second parameter to advertise() is the size of the message queue
      * used for publishing messages.  If messages are published more quickly
      * than we can send them, the number here specifies how many messages to
      * buffer up before throwing some away.
      */
     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

     ros::Rate loop_rate(10);

     /**
      * A count of how many messages we have sent. This is used to create
      * a unique string for each message.
      */
     int count = 0;
     while (ros::ok())
     {
       /**
        * This is a message object. You stuff it with data, and then publish it.
        */
       std_msgs::String msg;

       std::stringstream ss; 
       ss << "hello world " << count;
       msg.data = ss.str();

       ROS_INFO("%s", msg.data.c_str());

       /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
       */
      chatter_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }


    return 0;
  }

```

#### 代码说明

```C++
#include"ros/ros.h"
```

ros/ros.h是一个实用的头文件，引用了ROS系统中大部分常用的头文件。

```C++
#include"std_msgs/String.h"
```

引用std_msgs/String消息，存放在std_msgs package里，是由String.msg文件自动生成的头文件。

```C++
ros::init(argc, argv, "talker");
```

初始化ROS。允许ROS通过命令进行名称重映射，在运行过程中，节点的名称必须唯一。名称必须是一个base name，即名称内不能包含/等符号。

```C++
ros::NodeHandle n;
```

为这个进程的节点创建一个句柄。第一个创建的NodeHandle会为节点进行初始化，最后一个销毁的NodeHandle则会释放该节点所占用的所有资源。

```c++
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```

告诉master要在chatter话题上发布std_msgs/String消息类型的消息。第二个参数是发布序列的大小。

NodeHandle::advertise()返回一个ros::Publisher对象，有两个作用：1）有一个publish()成员函数可以让你在topic上发布消息；2）如果消息类型不对，会拒绝发布。

```c++
ros::Rate loop_rate(10);
```

ros::Rate对象允许指定自循环的频率。它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间。

```c++
int count = 0;
while (ros::ok())
{
```

roscpp会默认生成一个SIGNT句柄，负责处理Ctrl-C键盘操作--使得ros::ok()返回false。

如果下列条件之一发生，ros::ok() 返回false：

- 列表项SIGINT 被触发 (Ctrl-C)
- 列表项被另一同名节点踢出 ROS 网络
- 列表项ros::shutdown() 被程序的另一部分调用
- 节点中的所有 ros::NodeHandles 都已经被销毁 一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。

```c++
std_msgs::String msg;

std::stringstream ss;
ss << "hello world " << count;
msg.data = ss.str();
```

使用一个由msg file文件产生的‘消息自适应’类在ROS网络中广播消息。

```C++
      chatter_pub.publish(msg);
```

向所有订阅chatter话题的节点发送消息。

```c++
 ROS_INFO("%s", msg.data.c_str());
```

ROS_INFO 和其他类似的函数可以用来代替 printf/cout 等函数。

```c++
 loop_rate.sleep();
```

调用 ros::Rate 对象来休眠一段时间以使得发布频率为 10Hz。

总结：

- 列表项初始化 ROS 系统
- 列表项在 ROS 网络内广播我们将要在 chatter 话题上发布 std_msgs/String 类型的消息
- 列表项以每秒 10 次的频率在 chatter 上发布消息

 ### 编写订阅器节点

#### 源代码

在 beginner_tutorials package 目录下创建 src/listener.cpp 文件。

```c++
#include "ros/ros.h"
   #include "std_msgs/String.h"

   /**
    * This tutorial demonstrates simple receipt of messages over the ROS system.
    */
   void chatterCallback(const std_msgs::String::ConstPtr& msg)
   {
     ROS_INFO("I heard: [%s]", msg->data.c_str());
   }

   int main(int argc, char **argv)
   {
     /**
      * The ros::init() function needs to see argc and argv so that it can perform
      * any ROS arguments and name remapping that were provided at the command line. For programmatic
      * remappings you can use a different version of init() which takes remappings
      * directly, but for most command-line programs, passing argc and argv is the easiest
      * way to do it.  The third argument to init() is the name of the node.
      *
      * You must call one of the versions of ros::init() before using any other
      * part of the ROS system.
      */
     ros::init(argc, argv, "listener");

     /**
      * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
      */
     ros::NodeHandle n;

     /**
      * The subscribe() call is how you tell ROS that you want to receive messages
      * on a given topic.  This invokes a call to the ROS
      * master node, which keeps a registry of who is publishing and who
      * is subscribing.  Messages are passed to a callback function, here
      * called chatterCallback.  subscribe() returns a Subscriber object that you
      * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
      * object go out of scope, this callback will automatically be unsubscribed from
      * this topic.
      *
      * The second parameter to the subscribe() function is the size of the message
      * queue.  If messages are arriving faster than they are being processed, this
      * is the number of messages that will be buffered up before beginning to throw
      * away the oldest ones.
      */
     ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

     /**
      * ros::spin() will enter a loop, pumping callbacks.  With this version, all
      * callbacks will be called from within this thread (the main one).  ros::spin()
      * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
      */
     ros::spin();

     return 0;
   }
```

#### 代码说明

```c++
 void chatterCallback(const std_msgs::String::ConstPtr& msg)
   {
     ROS_INFO("I heard: [%s]", msg->data.c_str());
   }
```

回调函数，当接收到chatter话题的时候会被调用。消息以boost shared_ptr指针的形式传输。

```c++
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
```

当有消息发布到chatter话题上时，ROS就会调用chatterCallback()函数。第二个参数是队列大小。

NodeHandle::subscribe()返回ros::Subscriber对象，必须使其处于活动状态直到不再订阅该消息。当这个对象销毁时，会自动退订chatter话题的消息。

```c++
ros::spin();
```

ros::spin()进入自循环，可以尽可能快的调用消息回调函数。如果没有消息到达，它不会占用很多 CPU。一旦 ros::ok() 返回 false，ros::spin() 就会立刻跳出自循环。

总结：

- 列表项初始化ROS系统
- 列表项订阅 chatter 话题
- 列表项进入自循环，等待消息的到达
- 列表项当消息到达，调用 chatterCallback() 函数

## 编写服务器和客户端(C++)

简介：介绍如何用C++编写服务器Service和客户端Client节点。

### 编写Service节点

创建一个简单的service节点add_two_ints_server。

在 beginner_tutorials 包中创建`src/add_two_ints_server.cpp`文件。

```c++
 #include "ros/ros.h"
    #include "beginner_tutorials/AddTwoInts.h"

    bool add(beginner_tutorials::AddTwoInts::Request  &req,
             beginner_tutorials::AddTwoInts::Response &res)
    {
      res.sum = req.a + req.b;
      ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
      ROS_INFO("sending back response: [%ld]", (long int)res.sum);
      return true;
    }

    int main(int argc, char **argv)
    {
      ros::init(argc, argv, "add_two_ints_server");
      ros::NodeHandle n;

      ros::ServiceServer service = n.advertiseService("add_two_ints", add);
      ROS_INFO("Ready to add two ints.");
      ros::spin();

      return 0;
    }
```

### 编写Client节点

在 beginner_tutorials 包中创建`src/add_two_ints_client.cpp`文件。

```c++
    #include "ros/ros.h"
    #include "beginner_tutorials/AddTwoInts.h"
    #include <cstdlib>

    int main(int argc, char **argv)
    {
      ros::init(argc, argv, "add_two_ints_client");
      if (argc != 3)
      {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
      }

      ros::NodeHandle n;
      ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
       beginner_tutorials::AddTwoInts srv;
       srv.request.a = atoll(argv[1]);
       srv.request.b = atoll(argv[2]);
       if (client.call(srv))
       {
         ROS_INFO("Sum: %ld", (long int)srv.response.sum);
       }
       else
       {
        ROS_ERROR("Failed to call service add_two_ints");
         return 1;
       }

      return 0;
     }
```

