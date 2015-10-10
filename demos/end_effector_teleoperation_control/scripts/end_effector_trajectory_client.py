#!/usr/bin/env python
import rospy
import struct
import sys
#import pdb
from copy import copy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.msg import (
    JointCommand
)

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

isMoving = False

def callback(data):
        global isMoving
        if (isMoving):
            print "But last command isn't finished.This command will be ignored."
            return 0

        current_limb = "right"
        if not(data.names[0].find("left") == -1):
            current_limb = "left"
        isMoving = True
        limb_interface = baxter_interface.limb.Limb(current_limb)
        
        goal_angles = data.command[:]
        joint_names = [current_limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        jointCommand = dict()
        for i in range(0, 7):
            jointCommand[joint_names[i]] = goal_angles[i]

        limb_interface.set_joint_positions(jointCommand)
        isMoving = False
        return 0

    
def listener():
    print("Initializing node end_effector_trajectory_client... ")
    rospy.init_node('end_effector_trajectory_client', anonymous=True)
    print("Getting robot state... ")
    fine = False
    rate = rospy.Rate(1)
    while not fine | rospy.is_shutdown():
        try:
            # rs = baxter_interface.RobotEnable(CHECK_VERSION)
            rs = baxter_interface.RobotEnable()
            print("Enabling robot... ")
            rs.enable()
            fine = True
        except(OSError):
            print("Can not enable robot.Will keep trying...")
            rate.sleep()
    if rospy.is_shutdown():
        return
    print("moving to neutral... ")
    right_arm = baxter_interface.limb.Limb("right")
    right_arm.move_to_neutral()
    left_arm = baxter_interface.limb.Limb("left")
    left_arm.move_to_neutral()
    print("end_effector_trajectory_client subscribing...")
    rospy.Subscriber("end_effector_command_solution", JointCommand, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()