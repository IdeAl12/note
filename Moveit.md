[TOC]

# MoveIt!

## Concepts

### System Architecture

![Overview.0012](/Users/lixiang/Downloads/Overview.0012.jpg)

### The move_group node

The figure above shows the high-level system architecture for the primary node provided by Moveit! called move_group. 

### User Interface

The users can access the actions and services provided by move_group in one of three ways:

- In C++:move_group_interface
- In Python: MoveIt_commander
- Through a GUI:Motion Planning plugin to Rviz

### Configuration

move_group is a ROS node. It uses the ROS param server to get three kinds of information:

- URDF
- SRDF
- MoveIt! configuration

### Robot Interface

move_group talks to the robot through ROS topics and actions.

### Joint State Information

move_group listens on the /joint_states topic for determining the current state information.

### Transform Information

move_group monitors transform information using the ROS RF library. This allows the node to get global information about the robot's pose.

### Controller Interface

move_group talks to the controllers on the robot using the FollowJointTrajectoryAction interface.

### Planning Scene

move_group uses the Planning scene Monitor to maintain a planning scene, which is a representation of the world and the current state of the robot.

# Tutorials

## Beginner 

The primary user interface to MoveIt! is through the move_group_interface. You can use theis interface both through C++ and python. A GUI-based interface is available through the use of the MoveIt! Rviz Plugin. 

- MoveIt! RViz Plugin Tutorial
- Move Group Interface Tutorial
- Move Group Python Interface Tutorial

## MoveIt! RViz Plugin Tutorial

## Move Group Interface Tutorial

In MoveIt!, the primary user interface is through the MoveGroup class. It provides easy to use functionality for most operations that a user may want to carry out, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot.

### Setup

The MoveGroup class can be setup using the name of the group you would like to control and plan for 

```
moveit::planning_interface::MoveGroup group("right_arm")
```

We use the PlanningSceneInterface class to deal directly with the world.

```
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
```

(Optional)Create a publisher for visualizing plans in Rviz.

```c++
ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
moveit_msgs::DisplayTrajectory display_trajectory;
```

### Getting Basic Information

