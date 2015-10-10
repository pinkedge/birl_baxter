#!/usr/bin/env python

import argparse
import copy

import rospy
import numpy

import baxter_interface
import baxter_external_devices
import tf

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from baxter_core_msgs.msg import (
    JointCommand
)
from std_msgs.msg import (
    String,
    Header,
)
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
)

pub = rospy.Publisher("end_effector_command_position", PoseStamped, queue_size=1)

current_limb = "right"
global_distance = 0.01
def callback(data):
    global current_limb
    global global_distance
    command = data.data
    if not (command.find("switch") == -1):
        if (command.find("right") == -1):
            current_limb = "left"
        else:
            current_limb = "right"
        return
    limb = baxter_interface.Limb(current_limb)

    listener = tf.TransformListener()
    while (True):
        try:
            (trans,rot) = listener.lookupTransform('/right_arm_mount', '/right_gripper', rospy.Time(0))
            break;
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    x = 0
    y = 0
    z = 0
    rx = 0
    ry = 0
    rz = 0
    rw = 0
    if (command == "up"):
        z = global_distance
    elif (command == "down"):
        z = -global_distance
    elif (command == "left"):
        y = global_distance
    elif (command == "right"):
        y = -global_distance
    elif (command == "backward"):
        x = -global_distance
    elif (command == "forward"):
        x = global_distance
    elif (command == "orientation_x"):
        rx = global_distance
    elif (command == "keep"):
        pass
    elif (command == "further"):
        if (global_distance < 0.3):
            global_distance += 0.05
            rospy.loginfo(global_distance)
        else:
            rospy.loginfo("can not increase more")
    elif (command == "closer"):
        if (global_distance > 0):
            global_distance -= 0.05
            if (global_distance < 0):
                global_distance = 0
            rospy.loginfo(global_distance)
        else:
            rospy.loginfo("can not decrease more")
    else:
        rospy.loginfo("unknown command")
        return
    newPose = Pose()
    newPose.position = Point(
                trans[0] + x,
                trans[1] + y,
                trans[2] + z,
            )
    newPose.orientation = Quaternion(
                rot[0] + rx,
                rot[1],
                rot[2],
                rot[3],
            )

    newHeader = Header()
    newHeader.frame_id = current_limb

    newHeader.stamp = rospy.Time().now()

    newPoseStamp = PoseStamped()
    newPoseStamp.pose = newPose
    newPoseStamp.header = newHeader

    pub.publish(newPoseStamp)

def subscribe():
    rospy.Subscriber("/end_effector_command", String, callback)
    rospy.loginfo("subscribing..")
    rospy.spin();
def main():
    rospy.loginfo("Initializing node ikfast_transform... ")
    rospy.init_node("ikfast_transform", anonymous=True)

    try:
        subscribe()
    except():
        pass
    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()
