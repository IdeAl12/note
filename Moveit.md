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

```python
diisplay_trajectory_publisher = rospy.Publisher(
									 '/move_group/display_planned_path',
									 moveit_msgs.msg.DisplayTrajectory)
```

Wait for Rviz to initialize.

```python
print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting tutorial "
```

### Getting Basic Information

```python
print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Reference frame: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
print "============"
```

### Planning to a Pose goal

```python
print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.7
pose_target.position.y = -0.05
pose_target.position.z = 1.1
group.set_pose_target(pose_target)
```

Call the planner to compute the plan and visualize it.

```python
plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)
print "============ Visualizing plan1"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

print "============ Waiting while plan1 is visualized (again)..."
rospy.sleep(5)
```

### Moving to a pose goal

Moving to a pose goal is similar to the step above execpt we now use the go() function.

```python
# Uncomment below line when working with a real robot
# group.go(wait=True)
```

## Kinematic Model Tutorial

Through the C++ API for using kinematics.

### The RobotModel and RobotState classes

The RobotModel and RobotState classes are the core classes that give you access to the kinematics.  

### Start

```C++
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");

const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
```

### Get Joint Values

We can retreive the current set of joint values stored in the state.

```c++
std::vector<double> joint_values;
kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
for(std::size_t i = 0; i < joint_names.size(); ++i)
{
  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
}
```

### Joint Limits

setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.

```C++
/* Set one joint in the right arm outside its joint limit */
joint_values[0] = 1.57;
kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

/* Check whether any joint is outside its joint limits */
ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

/* Enforce the joint limits for this state and check again*/
kinematic_state->enforceBounds();
ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
```

### Forward Kinematics

We can compute forward kinematics for a set of random joint values. 

```C++
kinematic_state->setToRandomPositions(joint_model_group);
const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");

/* Print end-effector pose. Remember that this is in the model frame */
ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
```

### Inverse Kinematics

To solve IK, wee will need the following: * The desired pose of the end_effector that we computed in the step above. * The number of attempts to be made at solving IK:5. * The timeout for each attempt: 0.1s.

```C++
bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effectoor, 10, 0.1);
```

Print out the IK solution (if found):

```C++
if (found_ik)
{
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i=0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
}
else
{
  ROS_INFO("Did not find IK solution");
}
```

### Get the Jacobian

We can get the Jacobian from the RobotState.

```C++
Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
Eigen::MatrixXd jacobian;
kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position,
                             jacobian);
ROS_INFO_STREAM("Jacobian: " << jacobian);
```

### The launch file

To run the code, you will need a launch file that does two things:

- Uplods the PR2 URDF and SRDF onto the param server.
- Puts the kinematics_solver configuration generated by the MoveIt! Setup Assistant onto the ROS parameter server in the namespace of the node that instantiates the classes.

```xml
  <launch>
    <include file="$(find pr2_moveit_config)/launch/planning_context.launch">
      <arg name="load_robot_description" value="true"/>
    </include>
  
    <node name="kinematic_model_tutorial"
          pkg="moveit_tutorials"
          type="kinematic_model_tutorial"
          respawn="false" output="screen">
      <rosparam command="load"
                file="$(find pr2_moveit_config)/config/kinematics.yaml"/>
    </node>
  </launch>
```

## Planning Scene Tutorial

The PlanningScene class provides the main interface that you will use for collision checking and constraint checking.

### Setup

PlanningSceneMonitor is the recommended method to create and maintain the current planning scene using data from the robot’s joints and the sensors on the robot. In this tutorial, 

```C++
robot_model_loader::RobotModelLoader 
robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
planning_scene::PlanningScene planning_scene(kinematic_model);
```

### Collision Checking

#### Self-collision checking

The first thing is check whether the robot in its current state is in self-collision, i.e. whether the current configuration of the robot would result in the robot's parts hitting each other.

```C++
collision_detection::CollisionRequest collision_request;
collision_detection::CollisionResult collision_result;
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 1: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");
```

#### Change the state

Note in particular that we need to clear the collision_result before making a new collision checking request.

```C++
robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
current_state.setToRandomPositions();
collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 2: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");
```

#### Checking for a group

Now, we will do collision checking only for the right_arm of the PR2.

```C++
collision_request.group_name = "right_arm";
current_state.setToRandomPositions();
collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 3: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");
```

#### Getting Contact Information

First, manually set the right arm to a position where internal(self) collisions do happen. Note that this state is now actually outside the joint limits of the PR2, which we can also check for directly.

```C++
std::vector<double> joint_values;
const robot_model::JointModelGroup* joint_model_group =
  current_state.getJointModelGroup("right_arm");
current_state.copyJointGroupPositions(joint_model_group, joint_values);
joint_values[0] = 1.57; //hard-coded since we know collisions will happen here
current_state.setJointGroupPositions(joint_model_group, joint_values);
ROS_INFO_STREAM("Current state is "
                << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
```

Get contact information for any collisions that might have happened at a given configuration of the right arm.

```C++
collision_request.contacts = true;
collision_request.max_contacts = 1000;

collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 4: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");
collision_detection::CollisionResult::ContactMap::const_iterator it;
for(it = collision_result.contacts.begin();
    it != collision_result.contacts.end();
    ++it)
{
  ROS_INFO("Contact between: %s and %s",
           it->first.first.c_str(),
           it->first.second.c_str());
}
```

#### Modifyign the Allowed Collision Matrix

The AllowedCollisionMatrix(ACM) provides a mechanism to tell the collision world to ignore collisions between certain object: both parts of the robot and objects in the world. We can tell the collision checker to ignore all collisions between the links reported above even through the links are actually in collision.

```C++
collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
robot_state::RobotState copied_state = planning_scene.getCurrentState();

collision_detection::CollisionResult::ContactMap::const_iterator it2;
for(it2 = collision_result.contacts.begin();
    it2 != collision_result.contacts.end();
    ++it2)
{
  acm.setEntry(it2->first.first, it2->first.second, true);
}
collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
ROS_INFO_STREAM("Test 5: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");
```

#### Full Collision Checking

We can use the checkCollision functions instead which will check for both self-collisions and for collisions with the environment.

```C++
collision_result.clear();
planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
ROS_INFO_STREAM("Test 6: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");
```

### Constraint Checking

The PlanningScene class also includes function calls for checking constraints. The constraints can be of two types: (a) constrains chosen from KinematicConstrain set. (b) user defined constraints specified through a callback. 

#### Checking Kinematic Constraints

First define a simple position and orientation constraint on the end-effector of the right_arm of the PR2 robot.

```C++
std::string end_effector_name = joint_model_group->getLinkModelNames().back();

geometry_msgs::PoseStamped desired_pose;
desired_pose.pose.orientation.w = 1.0;
desired_pose.pose.position.x = 0.75;
desired_pose.pose.position.y = -0.185;
desired_pose.pose.position.z = 1.3;
desired_pose.header.frame_id = "base_footprint";
moveit_msgs::Constraints goal_constraint =
  kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);
```

Now, we can check a state against this constraint using the isStateConstrained functions in the PlanningScene Class.

```C++
copied_state.setToRandomPositions();
copied_state.update();
bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
ROS_INFO_STREAM("Test 7: Random state is "
                << (constrained ? "constrained" : "not constrained"));
```

There's a more efficient way of checking constraints when you waant to check the same constraint over and over again.Construct a KinematicConstrainSet which pre-processes the ROS Constraints messages and sets it up for quick processing.

```C++
kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
bool constrained_2 =
  planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
ROS_INFO_STREAM("Test 8: Random state is "
                << (constrained_2 ? "constrained" : "not constrained"));
```

There’s a direct way to do this using the KinematicConstraintSet class.

```C++
kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
  kinematic_constraint_set.decide(copied_state);
ROS_INFO_STREAM("Test 9: Random state is "
                << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));
```

#### User-definde constraints

User defined constraints can also be specified to the PlanningScene class. This is done by specifying a callback using the setStateFeasibilityPredicate function.

```C++
bool userCallback(const robot_state::RobotState &kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("r_shoulder_pan_joint");
  return (joint_values[0] > 0.0);
}
```

Now, whenever isStateFeasible is called, this user-defined callback will be called.

```C++
planning_scene.setStateFeasibilityPredicate(userCallback);
bool state_feasible = planning_scene.isStateFeasible(copied_state);
ROS_INFO_STREAM("Test 10: Random state is "
                << (state_feasible ? "feasible" : "not feasible"));
```

Whenever isStateValid is called, three checks are conducted: (a) collision checking (b) constraint checking and (c) feasibility checking using the user-defined callback.

```C++
bool state_valid =
  planning_scene.isStateValid(copied_state, kinematic_constraint_set, "right_arm");
ROS_INFO_STREAM("Test 10: Random state is "
                << (state_valid ? "valid" : "not valid"));
```

## ROS API Planning Scene Tutorial

- **Adding and removing objects into the world**
- **Attaching and detaching objects to the robot**

### ROS API

The ROS API to the planning scene publisher is through a topic interface using "diffs". A planning scene diff is the difference between the current planning scene and the new planning scene desired by the user.

### Addvertise the required topic

Note that this topic may need to be remapped in the launch file.

```C++
ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
while(planning_scene_diff_publisher.getNumSubscribers() < 1)
{
  ros::WallDuration sleep_t(0.5);
  sleep_t.sleep();
}
```

### Define the attached object message

We will this message to add or subtract the object from the world and to attach the object to the robot.

```C++
moveit_msgs::AttachedCollisionObject attached_object;
attached_object.link_name = "r_wrist_roll_link";
/* The header must contain a valid TF frame*/
attached_object.object.header.frame_id = "r_wrist_roll_link";
/* The id of the object */
attached_object.object.id = "box";

/* A default pose */
geometry_msgs::Pose pose;
pose.orientation.w = 1.0;

/* Define a box to be attached */
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.1;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.1;

attached_object.object.primitives.push_back(primitive);
attached_object.object.primitive_poses.push_back(pose);
```

Note that attaching an object to the robot requires the corresponding operation to be specified as an ADD operation.

```C++
attached_object.object.operation = attached_object.object.ADD
```

### Add an object into the environment

Add the object into the environment by adding it to the set of collision objects in the “world” part of the planning scene. Note that we are using only the “object” field of the attached_object message here.

```C++
ROS_INFO("Adding the object into the world at the location of the right wrist.");
moveit_msgs::PlanningScene planning_scene;
planning_scene.world.collision_objects.push_back(attached_object.object);
planning_scene.is_diff = true;
planning_scene_diff_publisher.publish(planning_scene);
sleep_time.sleep();
```

### Attach an object to the robot

When thr robot picks up an object from the environment, we need to "attach" the object to the robot so that any component dealing with the robot model knows to account for the attached object.

Attaching an object requires two operations

- Removing the original object from the environment
- Attaching the object to the robot

```C++
/* First, define the REMOVE object message*/
moveit_msgs::CollisionObject remove_object;
remove_object.id = "box";
remove_object.header.frame_id = "odom_combined";
remove_object.operation = remove_object.REMOVE;
```

```C++
/* Carry out the REMOVE + ATTACH operation */
ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
planning_scene.world.collision_objects.clear();
planning_scene.world.collision_objects.push_back(remove_object);
planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
planning_scene_diff_publisher.publish(planning_scene);

sleep_time.sleep();
```

### Detach an object from the robot

Detaching an object from the robot requires two operations

- Detaching an object from the robot
- Re-introducing the object into the environment

```C++
/* First, define the DETACH object message*/
moveit_msgs::AttachedCollisionObject detach_object;
detach_object.object.id = "box";
detach_object.link_name = "r_wrist_roll_link";
detach_object.object.operation = attached_object.object.REMOVE;
```

```C++
/* Carry out the DETACH + ADD operation */
ROS_INFO("Detaching the object from the robot and returning it to the world.");
planning_scene.robot_state.attached_collision_objects.clear();
planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
planning_scene.world.collision_objects.clear();
planning_scene.world.collision_objects.push_back(attached_object.object);
planning_scene_diff_publisher.publish(planning_scene);

sleep_time.sleep();
```

### Remove the object from the collision world

Removing the object from the collision world just requires using the remove object message defined earlier. 

```C++
ROS_INFO("Removing the object from the world.");
planning_scene.robot_state.attached_collision_objects.clear();
planning_scene.world.collision_objects.clear();
planning_scene.world.collision_objects.push_back(remove_object);
planning_scene_diff_publisher.publish(planning_scene);
```

## Motion Planners Tutorial

In MoveIt!, the motion planners are loaded using a plugin infrastructure. This allows MoveIt! to load motion planners at runtime.

### Start

Planners are setup as plugins in MoveIt! and you can use the ROS pluginlib interface to load any planner that you want to use. Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.

```C++
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
```

Using the RobotModel, we can construct a PlanningScene that maintains the state of the world (including the robot).

```C++
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
```

We will now construct a loader to load a planner, by name. Note that we are using the ROS pluginlib library here.

```C++
boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
planning_interface::PlannerManagerPtr planner_instance;
std::string planner_plugin_name;
```

We will get the name of planning plugin we want to load from the ROS param server, and then load the planner making sure to catch all exceptions.

```C++
if (!node_handle.getParam("planning_plugin", planner_plugin_name))
  ROS_FATAL_STREAM("Could not find planner plugin name");
try
{
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
}
catch(pluginlib::PluginlibException& ex)
{
  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
}
try
{
  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
    ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
}
catch(pluginlib::PluginlibException& ex)
{
  const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
  std::stringstream ss;
  for (std::size_t i = 0 ; i < classes.size() ; ++i)
    ss << classes[i] << " ";
  ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                   << "Available plugins: " << ss.str());
}

/* Sleep a little to allow time to startup rviz, etc. */
ros::WallDuration sleep_time(15.0);
sleep_time.sleep();
```

### Pose Goal

```C++
planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;
geometry_msgs::PoseStamped pose;
pose.header.frame_id = "torso_lift_link";
pose.pose.position.x = 0.75;
pose.pose.position.y = 0.0;
pose.pose.position.z = 0.0;
pose.pose.orientation.w = 1.0;
```

A tolerance of 0.01 m is specified in position and 0.01 radians in orientation

```C++
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);
```

Create the request as a constraint using a helper function available from the kinematic_constraints package.

```C++
req.group_name = "right_arm";
moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
req.goal_constraints.push_back(pose_goal);
```

We now construct a planning context that encapsulate the scene, the request and the response. We call the planner using this planning context

```C++
planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
if(res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}
```

### Visualize the result

```C++
ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;

/* Visualize the trajectory */
ROS_INFO("Visualizing the trajectory");
moveit_msgs::MotionPlanResponse response;
res.getMessage(response);

display_trajectory.trajectory_start = response.trajectory_start;
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);

sleep_time.sleep();
```

### Joint Space Goals

```C++
/* First, set the state in the planning scene to the final state of the last plan */
robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
planning_scene->setCurrentState(response.trajectory_start);
const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("right_arm");
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
```

Setup a joint space goal.

```C++
robot_state::RobotState goal_state(robot_model);
std::vector<double> joint_values(7, 0.0);
joint_values[0] = -2.0;
joint_values[3] = -0.2;
joint_values[5] = -0.15;
goal_state.setJointGroupPositions(joint_model_group, joint_values);
moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
req.goal_constraints.clear();
req.goal_constraints.push_back(joint_goal);
```

Call the planner and visualize the trajectory

```C++
/* Re-construct the planning context */
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
/* Call the Planner */
context->solve(res);
/* Check that the planning was successful */
if(res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}
/* Visualize the trajectory */
ROS_INFO("Visualizing the trajectory");
res.getMessage(response);
display_trajectory.trajectory_start = response.trajectory_start;
display_trajectory.trajectory.push_back(response.trajectory);

/* Now you should see two planned trajectories in series*/
display_publisher.publish(display_trajectory);

/* We will add more goals. But first, set the state in the planning
   scene to the final state of the last plan */
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

/* Now, we go back to the first goal*/
req.goal_constraints.clear();
req.goal_constraints.push_back(pose_goal);
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
res.getMessage(response);
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);
```

### Adding Path Constraints

Add a new pose goal again. This time we will also add a path constraint to the motion.

```C++
/* Let's create a new pose goal */
pose.pose.position.x = 0.65;
pose.pose.position.y = -0.2;
pose.pose.position.z = -0.1;
moveit_msgs::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
/* First, set the state in the planning scene to the final state of the last plan */
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
/* Now, let's try to move to this new pose goal*/
req.goal_constraints.clear();
req.goal_constraints.push_back(pose_goal_2);

/* But, let's impose a path constraint on the motion.
   Here, we are asking for the end-effector to stay level*/
geometry_msgs::QuaternionStamped quaternion;
quaternion.header.frame_id = "torso_lift_link";
quaternion.quaternion.w = 1.0;
req.path_constraints = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", quaternion);
```

Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector, because of this, we need to specify a bound for the allowed planning volume as well.

```C++
req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.0;
req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z =  2.0;
```

Call the planner and visualize all the plans created so far.

```C++
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
res.getMessage(response);
display_trajectory.trajectory.push_back(response.trajectory);
```

Now you should see four planned trajectories in series

```C++
display_publisher.publish(display_trajectory);
```

