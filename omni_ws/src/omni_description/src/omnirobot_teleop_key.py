#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

#Define maximum velocities
MAX_LIN_VEL = 0.4
MAX_ANG_VEL = 1.0

#Define increment for each velocity
LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Omnidirectional mobile robot!
(To exit, press ctrl+C)
---------------------------
Moving around (LINEAR):
        w
   a    s    d
        x

w/x : increase/decrease X linear velocity (Max vel: 0.4)
a/d : increase/decrease Y linear velocity (Max vel: 0.4)

Rotating around (ANGULAR):
j/l : increase/decrease Z angular velocity (Max vel: 1.0)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

#Function to use keyboard interruption
def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_x_vel, target_y_vel, target_angular_vel):
    return "currently:\tX vel %s\tY vel %s\tZ vel %s " % (target_x_vel,target_y_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input


def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('omni_teleop_key', anonymous="True") #To use multiple times this node at the same time
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    status = 0

    target_x_vel = 0.0
    target_y_vel = 0.0
    target_angular_vel = 0.0

    control_x_vel = 0.0
    control_y_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w' :
                target_x_vel = checkLinearLimitVelocity(target_x_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_x_vel,target_y_vel,target_angular_vel))
            elif key == 'x' :
                target_x_vel = checkLinearLimitVelocity(target_x_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_x_vel,target_y_vel,target_angular_vel))
            elif key == 'a' :
                target_y_vel = checkLinearLimitVelocity(target_y_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_x_vel,target_y_vel,target_angular_vel))
            elif key == 'd' :
                target_y_vel = checkLinearLimitVelocity(target_y_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_x_vel,target_y_vel,target_angular_vel))

            elif key == 'j' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_x_vel,target_y_vel,target_angular_vel))
            elif key == 'l' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_x_vel,target_y_vel,target_angular_vel))

            elif key == ' ' or key == 's' :
                target_x_vel   = 0.0
                target_y_vel   = 0.0
                control_x_vel  = 0.0
                control_y_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_x_vel,target_y_vel,target_angular_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_x_vel = makeSimpleProfile(control_x_vel, target_x_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_y_vel = makeSimpleProfile(control_y_vel, target_y_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_x_vel; twist.linear.y = control_y_vel; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    finally: #Stop the robot
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
