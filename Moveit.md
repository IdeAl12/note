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

Print the name of the reference frame for this robot:

```C++
ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_srt());
```

Print the name of the end_effector link for this group:

```C++
ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
```

### Planning to a Pose goal

Plan a motion for this group to a desired pose for the end-effector.

```C++
geometry_msgs::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.28;
target_pose1.position.y = -0.7;
target_pose1.position.z = 1.0;
group.setPoseTarget(target_pose1);
```

Call the planner to compute the plan and visualize it. 

```C++
moveit::planning_interface::MoveGroup::Plan my_plan;
bool success = group.plan(my_plan);

ROS_INFO("Visualizing plan 1 (pose goal) %s", success?"":"FAILED");
sleep(5.0);
```

### Visualizing plans

Now that we have a plan we can visualize it in Rviz. This is not necessary because the group.plan() call we made above did this automatically. But explicity publishing plans is useful in cases that we want to visualize a previously created plan.

```c++
if (1)
{
    ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    sleep(5.0);
}
```

### Moving to a pose goal

Movong to a pose goal is similar to the step above except we use the move() function. 

```C++
group.move();
```

### Planning to a joint-space goal

First get the current set of joint values for the group.

```c++
std::vector<double> group_variable_values;
group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()).group_variable_values);
```

Plan to the new joint space goal and visualize the plan.

```C++
group_variable_values[0] = -1.0;
group.setJointValueTarget(group_variable_values);
success = group.plan(my_plan);
ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
sleep(5.0);
```

### Planning with Path Constraints

Path constraints can easily be specified for a link on the robot. First define the path constraint.

```C++
moveit_msgs::OrientationConstraint ocm;
ocm.link_name = "r_wrist_rool_link";
ocm.header.frame_id = "base_link"
ocm.orientation.w = 1.0;
ocm.absolute_x_axis_tolerance = 0.1;
ocm.absolute_y_axis_tolerance = 0.1;
ocm.absolute_z_axis_tolerance = 0.1;
ocm.weight = 1.0;
```

Set it as the path constraint for the group.

```c++
moveit_msgs::Constraints test_constraints;
test_constraints.orientation_constraints.push_back(ocm);
group.setPathConstraints(test_constraints);
```

This will only work if the current state already satisfies the path constraints. So, we need to set the start state to a new pose.

```c++
robot_state::RobotState start_state(*group.getCurrentState());
geometry_msgs::Pose start_pose2;
start_pose2.orientation.w = 1.0;
start_pose2.position.x = 0.55;
start_pose2.position.y = -0.05;
start_pose2.position.z = 0.8;
const robot_state::JointModelGroup *joint_model_group =
                start_state.getJointModelGroup(group.getName());
start_state.setFromIK(joint_model_group, start_pose2);
group.setStartState(start_state);
```

Plan to the earlier pose target from the new start state that we have just created.

```c++
group.setPoseTarget(target_pose1);
success = group.plan(my_plan);

ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
/* Sleep to give Rviz time to visualize the plan. */
sleep(10.0);
```

When done with the path constraint be sure to clear it.

```C++
group.clearPathConstraints();
```

### Cartesian Paths(笛卡尔路径)

You can plan a cartesian path directly by specifying a list of waypoints for the end-effector to go through. The initial pose (start state) does not need to be added to the waypoint list.

```c++
std::vector<geometry_msgs::Pose> waypoints;

geometry_msgs::Pose target_pose3 = start_pose2;
target_pose3.position.x += 0.2;
target_pose3.position.z += 0.2;
waypoints.push_back(target_pose3);  // up and out

target_pose3.position.y -= 0.2;
waypoints.push_back(target_pose3);  // left

target_pose3.position.z -= 0.2;
target_pose3.position.y += 0.2;
target_pose3.position.x -= 0.2;
waypoints.push_back(target_pose3);  // down and right (back to start)
```

We want the cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in cartesian translation. We will specify the jump threshold as 0.0, effectively disabling it.

```c++
moveit_msgs::RobotTrajectory trajectory;
double fraction = group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);

ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
      fraction * 100.0);
/* Sleep to give Rviz time to visualize the plan. */
sleep(15.0);
```

### Adding/Removing Objects and Attaching/Detaching Objects

First, define the collision object message.

```C++
moveit_msgs::CollisionObject collision_object;
collision_object.header.frame_id = group.getPlanningFrame();

/* The id of the object is used to identify it. */
collision_object.id = "box1";

/* Define a box to add to the world. */
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.4;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.4;

/* A pose for the box (specified relative to frame_id) */
geometry_msgs::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x =  0.6;
box_pose.position.y = -0.4;
box_pose.position.z =  1.2;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

std::vector<moveit_msgs::CollisionObject> collision_objects;
collision_objects.push_back(collision_object);
```

Let’s add the collision object into the world.

```C++
ROS_INFO("Add an object into the world");
planning_scene_interface.addCollisionObjects(collision_objects);

/* Sleep so we have time to see the object in RViz */
sleep(2.0);
```

```C++
group.setStartState(*group.getCurrentState());
group.setPoseTarget(target_pose1);
success = group.plan(my_plan);

ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
  success?"":"FAILED");
/* Sleep to give Rviz time to visualize the plan. */
sleep(10.0);
```

Let's attach the collision object to the robot.

```C++
ROS_INFO("Attach the object to the robot");
group.attachObject(collision_object.id);
/* Sleep to give Rviz time to show the object attached (different color). */
sleep(4.0);
```

Let’s detach the collision object from the robot.

```C++
ROS_INFO("Detach the object from the robot");
group.detachObject(collision_object.id);
/* Sleep to give Rviz time to show the object detached. */
sleep(4.0);
```

Remove the collision object from the world.

```C++
ROS_INFO("Remove the object from the world");
std::vector<std::string> object_ids;
object_ids.push_back(collision_object.id);
planning_scene_interface.removeCollisionObjects(object_ids);
/* Sleep to give Rviz time to show the object is no longer there. */
sleep(4.0);
```

## Move Group Python Interface Tutorial

### Setup

To use the python interface to move_group, import the moveit_commander modeule. 

```python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
```

First initialize moveit_commander and rospy.

```python
print "===== Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial.'
                anonymous=True)
```

Instantiate a RobotCommander object. This object is an interface to the robot as a whole.

```python
robot = moveit_commander.RobotCommander()
```

IInstatiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.

```python
scene = moveit_commander.PlanningSceneInterface()
```

Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the left arm.

```python
group = moveit_commander.MoveGroupCommander("left_arm")
```

Create this DisplayTrajectory publisher.

```
diisplay_trajectory_publisher = rospy.Publisher(
									 '/move_group/display_planned_path',
									 moveit_msgs.msg.DisplayTrajectory)
```

