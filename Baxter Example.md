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

