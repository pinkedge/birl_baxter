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

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

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
        return

        #traj = Trajectory(current_limb)
        #rospy.on_shutdown(traj.stop)
        # Command Current Joint Positions first
        #current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        #traj.add_point(current_angles, 0.0)
        traj.add_point(goal_angles, 2)
        traj.start()
        traj.wait(2.1)
        print("Exiting - Joint Trajectory Action Complete")
        isMoving = False
        return 0
    
def listener():
    print("Initializing node end_effector_trajectory_client... ")
    rospy.init_node('end_effector_trajectory_client', anonymous=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
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