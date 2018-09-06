# Baxter Example

## Joint Position Keyboard Example

### Summary 

The joint position keyboard example demonstrates basic joint position control. Each key of PC is mapped to either increasing or decreasing the angles of a particular joint on Baxter's arms. Each arm is represented by one side of the keyboard with corresponding increase/decrease joint key pairings within the rows.

### Usage

```
# Verify that the robot is enabled from an RSDK terminal session
$ rosrun baxter_tools enable_robot.py -e  (rbe)
# Start the joint position keyboard example program.
$ rosrun baxter_examples joint_position_keyboard.py
```

```
  key bindings: 
  Esc: Quit
  ?: Help
  /: left: gripper calibrate
  ,: left: gripper close
  m: left: gripper open
  y: left_e0 decrease
  o: left_e0 increase
  u: left_e1 decrease
  i: left_e1 increase
  6: left_s0 decrease
  9: left_s0 increase
  7: left_s1 decrease
  8: left_s1 increase
  h: left_w0 decrease
  l: left_w0 increase
  j: left_w1 decrease
  k: left_w1 increase
  n: left_w2 decrease
  .: left_w2 increase
  b: right: gripper calibrate
  c: right: gripper close
  x: right: gripper open
  q: right_e0 decrease
  r: right_e0 increase
  w: right_e1 decrease
  e: right_e1 increase
  1: right_s0 decrease
  4: right_s0 increase
  2: right_s1 decrease
  3: right_s1 increase
  a: right_w0 decrease
  f: right_w0 increase
  s: right_w1 decrease
  d: right_w1 increase
  z: right_w2 decrease
  v: right_w2 increase
```

### Code Walkthrough

```python
import argparse 
import rospy
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
# This imports the baxter interface for accessing the limb and the gripper class. The baxter_external_devices is imported to use its getch function, that captures the key presses on the keyboard. It is not necessary to check the version in custom programs.

def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left')
    grip_right = baxter_interface.Gripper('right')
    lj = left.joint_names()
    rj = right.joint_names()
    # Two instances of the Limb class are created for each of Baxter's limbs. Similarly two instances of the Gripper class. joint_names() method returns an array of joints in that limb.

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)
    # The set_j() function is invoked whenever a valid key press occurs. The limb refers to the limb instance of baxter's limbs for the corresponding side. delta refers to the required displacement of that joint. The the joint command message is updated for that corresponding joint to indicate the new position. set_joint_positions() method published the joint commands to the position controller.
     bindings = {
     #   key: (function, args, description)
         '9': (set_j, [left, lj[0], 0.1], "left_s0 increase"),
         '6': (set_j, [left, lj[0], -0.1], "left_s0 decrease"),
         '8': (set_j, [left, lj[1], 0.1], "left_s1 increase"),
         '7': (set_j, [left, lj[1], -0.1], "left_s1 decrease"),
         'o': (set_j, [left, lj[2], 0.1], "left_e0 increase"),
         'y': (set_j, [left, lj[2], -0.1], "left_e0 decrease"),
         'i': (set_j, [left, lj[3], 0.1], "left_e1 increase"),
         'u': (set_j, [left, lj[3], -0.1], "left_e1 decrease"),
         'l': (set_j, [left, lj[4], 0.1], "left_w0 increase"),
         'h': (set_j, [left, lj[4], -0.1], "left_w0 decrease"),
         'k': (set_j, [left, lj[5], 0.1], "left_w1 increase"),
         'j': (set_j, [left, lj[5], -0.1], "left_w1 decrease"),
         '.': (set_j, [left, lj[6], 0.1], "left_w2 increase"),
         'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
         ',': (grip_left.close, [], "left: gripper close"),
         'm': (grip_left.open, [], "left: gripper open"),
         '/': (grip_left.calibrate, [], "left: gripper calibrate"),
 
         '4': (set_j, [right, rj[0], 0.1], "right_s0 increase"),
         '1': (set_j, [right, rj[0], -0.1], "right_s0 decrease"),
         '3': (set_j, [right, rj[1], 0.1], "right_s1 increase"),
         '2': (set_j, [right, rj[1], -0.1], "right_s1 decrease"),
         'r': (set_j, [right, rj[2], 0.1], "right_e0 increase"),
         'q': (set_j, [right, rj[2], -0.1], "right_e0 decrease"),
         'e': (set_j, [right, rj[3], 0.1], "right_e1 increase"),
         'w': (set_j, [right, rj[3], -0.1], "right_e1 decrease"),
         'f': (set_j, [right, rj[4], 0.1], "right_w0 increase"),
         'a': (set_j, [right, rj[4], -0.1], "right_w0 decrease"),
         'd': (set_j, [right, rj[5], 0.1], "right_w1 increase"),
         's': (set_j, [right, rj[5], -0.1], "right_w1 decrease"),
         'v': (set_j, [right, rj[6], 0.1], "right_w2 increase"),
         'z': (set_j, [right, rj[6], -0.1], "right_w2 decrease"),
         'c': (grip_right.close, [], "right: gripper close"),
         'x': (grip_right.open, [], "right: gripper open"),
         'b': (grip_right.calibrate, [], "right: gripper calibrate"),
      }
 # The bindings is a dictionary that holds the set of characters in the keyboard and their corresponding joints.

	
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
         	#catch Esc or ctrl-c
         	if c in ['\x1b', '\x03']:
         		done = True
         		rospy.signal_shutdown("xample finished.")
    # The done variable captures whether "esc" or "ctrl-c" was hit. The while loop iterates as long as the "esc" or "ctrl-c" is hit or ros-shutdown signal is given. c captures the keyboard input. 
    		 else:
                 print("key bindings: ")
                 print("  Esc: Quit")
                 print("  ?: Help")
                 for key, val in sorted(bindings.items(),
                                        key=lambda x: x[1][2]):
                     print("  %s: %s" % (key, val[2]))
                        
def main():
    epilog = """
    See help inside the example with th '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formater_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])
    
    print("Initializing node...")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    # init_state captures Baxter's initial state. This is to make sure that Baxter is sent back to the same state after the program exits.
	
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    print("Enabling robot...")
    rs.enable()
    map_keyboard()
    print("Done.")
    
if __name__ == '__main__':
    main()
           
```

## Display an image to Baxter's head

Display an image (e.g. .png or .jpg) to Baxter's head display. Baxter display resolution is 1024 * 600 pixels.

```
$ rosrun baxter_examples xdisplay_image.py --file=`rospack find baxter_examples`/share/images/baxterworking.png
# Replace the --file argument with the path to your own image.
```

### Code Walkthrough

```python
import os
import sys
import argparse

import rospy

import cv2
import cv_bridge
# The cv2 library is imported to read images as cv2 messages and perform various processing on it. The cv_bridge is used to convert between OpenCV and ROS images.
from sensor_msgs.msg import (
	image,
)

def send_image(path):
    im = cv2.imread(path)
    # The imread() is an OpenCV function that is used to load images as 2D numpy arrays. It is important to read the images as 2D numpy arrays in OpenCV 2.0 in order to execute various OpenCV algorithms on them.
    msg = cv_bridge.cvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    # The OpenCV image has to be converted into ROS image message in order to send them as ROS messages or perform any ROS functions on it. The cv_bridge is used to convert between ROS and OpenCV images. The cv2_to_image converts the OpenCV image to ROS image.
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    # The ROS image is published to the /robot/xdisplay topic
    pub.publish(msg)
    # Sleep to allow for image to be published
    rospy.sleep(1)
def main():
    # Pass the relative or absolut file path to an image file on your computer, and the example will read and convert the image using cv_bridge, sending it to the screen as a standard ROS Image Message.
    epilog = 
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
    	'-f', '--file', metavar='PATH', required=True,
    	help='Path to image file to send'
    )
    parser.add_argument(
    	'-d', '--delay', metavar='SEC', type=float, default=0.0,
    	help='Time in seconds to wait before publishing image'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    # The path of the image to be loaded, and the delay in publishing the images are captured as user arguments.
    rospy.init_node('rsdk_xdisplay_image', anonymous=True)
    
    if not os.access(args.file, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (args.file,))
        return 1
    # The rospy node is initialized. Then the readability of the path entered by the user is verified.
    if args.delay > 0:
        rospy.loginfo(
        	"Waiting for $s second(s) before publishing image to face" % (args.delay,)
        )
        rospy.sleep(args.delay)
        
    send_image(args.file)
    return 0

if __name__ == '__main__':
    sys.exit(main())
```



## Gripper Control 

Uses the keyboard or joystick to control Baxters grippers. Position, velocity, holding, and moving force can be controlled and sensed. Both logitech and xbox game controllers are supported.

```
$ rosrun baxter_examples gripper_keybiard.py
# You will have to calibrate both grippers before using any of the other commands using C/c commands.

# You can monitor the changes you are making using the following rostopics which you can monitor from a different shell:
$ rostopic echo /robot/end_effector/left_gripper/command
$ rostopic echo /robot/end_effector/right_gripper/command
```

### Code Walthrough

```python
import argparse
import rospy

import baxter_interface
import baxter_external_devices
# This imports the baxter interface for accessing the limb and the gripper class. The baxter_external_devices is imported to use its getch function that captures the key presses on the keyboard.
from baxter_interface import CHECK_VERSION

def map_keyboard():
    # initialize interfaces
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    right = baxter_interface.Gripper('right', CHECK_VERSION)
    # The init_state variable captures the current state of the robot. 
    def clean_shutdown():
        if not init_state:
            print("Disabling robot...")
            rs.disable()
        print("Exiting example.")
    rospy.on_shutdown(clean_shutdown)
    # On shutdown request, Baxter's state is sent back to its initial state.
    def capability_warning(gripper, cmd):
        msg = ("%s %s - not capable of '%s' command" %
               (gripper.name, gripper.type(), cmd))
        rospy.logwarn(msg)
        # This function is invoked when a gripper control command is called with custom grippers. It displays a warning message.
    def offset_position(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'command_position')
            return
        current = gripper.position()
        gripper.command_position(current + offset)
        # The gripper tyoe is checked for an electric gripper. The command_position() sets the gripper to the commanded position. It moves the fingers from close(0) to open(100) state based on the passed double vale.
	def offset_holding(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_holding_force')
            return
        current = gripper.parameters()['holding_force']
        gripper.set_holding_force(current + offset)
        # set_holding_force() method is used to set the force of the grippers when the fingers come in contact with an object between them.
	def offset_moving(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_moving_force')
            return
        current = gripper.parameters()['moving_force']
        gripper.set_moving_force(current + offset)
        # set_moving_force() method is used to set the force of the grippers immediately before the fingers come in contact with an object between them.
	def offset_velocity(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_velocity')
            return
        current = gripper.parameters()['velocity']
        gripper.set_velocity(current + offset)
        # set_velocity() method is used to set the velocity at which the grippers would open/close.
	def offset_dead_band(gripper. offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_dead_band')
            return
        current = gripper.parameters()['dead_zone']
        gripper.set_dead_band(current + offset)
        # set_dead_band() method is used to set the dead zone of the grippers. Precisely, it refers to the minimum distance between the gripper fingers when they are in closed state.
	 bindings = {
     #   key: (function, args, description)
         'r': (left.reboot, [], "left: reboot"),
         'R': (right.reboot, [], "right: reboot"),
         'c': (left.calibrate, [], "left: calibrate"),
         'C': (right.calibrate, [], "right: calibrate"),
         'q': (left.close, [], "left: close"),
         'Q': (right.close, [], "right: close"),
         'w': (left.open, [], "left: open"),
         'W': (right.open, [], "right: open"),
         '[': (left.set_velocity, [100.0], "left:  set 100% velocity"),
         '{': (right.set_velocity, [100.0], "right:  set 100% velocity"),
         ']': (left.set_velocity, [30.0], "left:  set 30% velocity"),
         '}': (right.set_velocity, [30.0], "right:  set 30% velocity"),
         's': (left.stop, [], "left: stop"),
         'S': (right.stop, [], "right: stop"),
         'z': (offset_dead_band, [left, -1.0], "left:  decrease dead band"),
         'Z': (offset_dead_band, [right, -1.0], "right:  decrease dead band"),
         'x': (offset_dead_band, [left, 1.0], "left:  increase dead band"),
         'X': (offset_dead_band, [right, 1.0], "right:  increase dead band"),
         'f': (offset_moving, [left, -5.0], "left:  decrease moving force"),
         'F': (offset_moving, [right, -5.0], "right:  decrease moving force"),
         'g': (offset_moving, [left, 5.0], "left:  increase moving force"),
         'G': (offset_moving, [right, 5.0], "right:  increase moving force"),
         'h': (offset_holding, [left, -5.0], "left:  decrease holding force"),
         'H': (offset_holding, [right, -5.0], "right:  decrease holding force"),
         'j': (offset_holding, [left, 5.0], "left:  increase holding force"),
         'J': (offset_holding, [right, 5.0], "right:  increase holding force"),
         'v': (offset_velocity, [left, -5.0], "left:  decrease velocity"),
         'V': (offset_velocity, [right, -5.0], "right:  decrease velocity"),
         'b': (offset_velocity, [left, 5.0], "left:  increase velocity"),
         'B': (offset_velocity, [right, 5.0], "right:  increase velocity"),
         'u': (offset_position, [left, -15.0], "left:  decrease position"),
         'U': (offset_position, [right, -15.0], "right:  decrease position"),
         'i': (offset_position, [left, 15.0], "left:  increase position"),
         'I': (offset_position, [right, 15.0], "right:  increase position"),
     }
    
    done = False
    print("Enabling robot...")
    rs.enable()
    print("Controlling grippers.Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            if c in ['\x1b', '\x03']:
                done = True
        elsif c in bindings:
            cmd = bindings[c]
            cmd[0](*cmd[1])
            print("command: %s" % (cmd[2],))
        else:
            print("key bindings: ")
            print("  Esc: Quit")
            print("  ?: Help")
            for key, val in sorted(bindings.items(),
                                   key=lamda x: x[1][2]):
                print("  %s: %s" % (key, val[2]))
	rospy.signal_shutdown("Example finished.")

def main():
    epilog = 
    arg_fmt = argsparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])
    
    print("Initializing node... ")
    rospy.init_node("rsdk_gripper_keyboard")
    map_keyboard()
    
if __name__ == '__main__':
    main()
        
```



## Head Movement Example

The Head "Wobbler" Example randomly moves the head to demonstrate using the hand pan and nod interfaces.

Baxter's head can move up and down in a nod_motion, and rotate left-to-right in a pan_motion. The head nod motion is a single action that can be triggered to run, but cannot be set to a specific angle. The pand motion swings Baxters 'face' to a settable angle, rotating around the vertical axes of the head.

```
$ rosrun baxter_examples head_wobbler.py
# Baxter's head will nod and begin panning left and right to random angles.
```

### baxter_interafce APIs

- Head class: head.py
- command_nod()
- set_pan(<angle>)

### Code Walkthrough

```python
import argparse
import random

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

class Wobbler(object):
    def __init__(self):
        self._done = False
        self._head = baxter_interface.Head()
        
```



## Head Action Client Example

The Head Action Client Example moves the head through a set of head positions and velocities to showcase an interface for the Head Action Server.

```
$ rosrun baxter_interface head_action_server.py
$ rosrun baxter_examples head_action_client.py
```

