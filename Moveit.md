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

In MoveIt!, the primary user interface is through the MoveGroup class.