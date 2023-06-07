#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from nav_msgs.msg import Path
import tf
from geometry_msgs.msg import PoseStamped

# Robot parameters
Lx = 0.08
Ly = 0.12
R = 0.04

# Global objects and message instances for publishing and subscribing
w1_r1_pub = rospy.Publisher("/omni/wheel1_joint_vel_contr/command", Float64, queue_size=1)
w2_r1_pub = rospy.Publisher("/omni/wheel2_joint_vel_contr/command", Float64, queue_size=1)
w3_r1_pub = rospy.Publisher("/omni/wheel3_joint_vel_contr/command", Float64, queue_size=1)
w4_r1_pub = rospy.Publisher("/omni/wheel4_joint_vel_contr/command", Float64, queue_size=1)

pose_sub1 = None

pose_1 = PoseStamped()

path_pub_1 = rospy.Publisher("/omni/path", Path, queue_size=10)

path_msg_1 = Path()

# The "omni1/odom" frame is chosen as the global coordinate frame for ALL robots
the_frame = "omni1/odom"  # In OdomCallbackR2, the tree tf is setup, that is, omni2/odom = omni1/odom

def odom_callback_r1(msg):
    global pose_1, path_msg_1

    pose_1.pose.position = msg.pose.pose.position
    path_msg_1.poses.append(pose_1)

    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    wz = msg.twist.twist.angular.z

    w1_r1_msg = (vx - vy - (Lx + Ly) * wz) / R
    w2_r1_msg = (vx + vy + (Lx + Ly) * wz) / R
    w3_r1_msg = (vx - vy + (Lx + Ly) * wz) / R
    w4_r1_msg = (vx + vy - (Lx + Ly) * wz) / R

    w1_r1_pub.publish(Float64(w1_r1_msg))
    w2_r1_pub.publish(Float64(w2_r1_msg))
    w3_r1_pub.publish(Float64(w3_r1_msg))
    w4_r1_pub.publish(Float64(w4_r1_msg))

def main():
    rospy.init_node("omni_tf_4wheels")  # Node name
    rate = rospy.Rate(50)  # Node frequency (Hz)

    global pose_sub1
    pose_sub1 = rospy.Subscriber("/odom", Odometry, odom_callback_r1)

    while not rospy.is_shutdown():
        path_pub_1.publish(path_msg_1)
        rospy.spin()
        rate.sleep()

if __name__ == "__main__":
    try:
        print("To finish this node, press Ctrl+C")
        main()
    except rospy.ROSInterruptException:
        pass

print("\nNode finished")

