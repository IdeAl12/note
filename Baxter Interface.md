# Baxter Interface

## What is the Baxter Interface (baxter_interface repository)

The baxter_interface repository holds our python API for interacting with the Baxter Research Robot. This comprises of a set of classes that provide wrappers around the ROS communications from Baxter, allowing for direct python control of the different interfaces of the robot.

## Available Interfaces

The Baxter_interface classes are available in the baxter_interface repository at baxter_interface/src/baxter_interface/.

### Limb

```python
from baxter_interface import Limb

right_arm = Limb('right')
left_arm = Limb('left')
```

Its main uses are:

- Querying the joint states
- Switching between control modes
- Sending Joint Commands (position, velocity, or torque)

### Gripper

```python
from baxter_interface import Gripper

right_gripper = Gripper('right')
left_gripper = Gripper('left')
```

Its main uses are:

- Sending open/close commands to the gripper
- Querying the state/properties of the gripper
- Reacting to grippers being plugged/unplugged
- Calibrating the gripper
- Controlling aspectsof how the gripper acts (velocity, moving force, holding force, dead band, vacuum threshold, etc.)

### Navigator 

```python
from baxter_interface import Navigator

right_arm_navigator = Navigator('right')
left_arm_navigator = Navigator('left')
right_torso_navigator = Navigator('torso_right')
left_torso_navigator = Navigator('torso_left')
```

Its main uses are:

- Querying the state of the wheel
- Responding to wheel and button interactions
- Controlling the navigator lights

### Robot Enable

```python
from baxter_interface import RobotEnable

baxter = RobotEnable
```

Its main uses are:

- Performing Enable, Disable, Stop, Reset on the robot
- getting the current robot state
- Verify the version of the software

### Camera

```python
from baxter_interface import CameraController

left_hand_camera = CameraController('left_hand_camera')
right_hand_camera = CameraController('right_hand_camera')
head_camera = CameraController('head_camera')
```

Its main uses are:

- Opening/closing cameras
- Updating camera resolution to another valid resolution mode
  - Valid modes:
    - (1280, 800)
    - (960, 600)
    - (640, 400)
    - (480, 300)
    - (384, 240)
    - (320, 200)
- Getting /Setting camera setting  (fps, exposure, gain, white balance, etc)

### Analog IO

```python
from baxter_interface import AnalogIO

<component name> = AnalogIO(<component id>)
```

Available Analog Components:

- left_hand_range, right_hand_range
- left_itb_wheel, right_itb_wheel, torso_left_itb_wheel, torso_right_itb_wheel
- left_vacuum_sensor_analog, right_vacuum_sensor_analog
- torso_fan

Available options:

- _on_io_state: react to state changes
- state
- is_output: check to see if the component is capable of output
- set_output

### Digital IO

```python
from baxter_interface import DigitalIO

<component name> = DigitalIO(<component id>)
```

Available options:

- _on_io_state: react to state changes
- state
- is_output: check to see if the component is capable of output
- set_output

 ### Head

```python
from baxter_interface import Head

<component name> = Head()
```

Available commands:

- _on_head_state: respond to state changes
- pan, nodding, panning: state values
- set_pan, command_nod