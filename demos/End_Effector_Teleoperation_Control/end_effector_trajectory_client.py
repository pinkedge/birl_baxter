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

def callback(data):
        print data
        #return
        #traj = Trajectory("right")
        #rospy.on_shutdown(traj.stop)
        # Command Current Joint Positions first
        limb_interface = baxter_interface.limb.Limb("right")
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        goal_angles = list()
        for i in range(0, 6):
            goal_angles.append(current_angles[i] + data.command[i])
        print goal_angles
        command = dict()
        for i in range(0, 6):
            command[data.names[i]] = goal_angles[i]
        print command
        limb_interface.move_to_joint_positions(command, 2)
        return

        traj.add_point(current_angles, 0.0)
        traj.add_point(goal_angles, 1.0)
        traj.start()
        traj.wait(2.0)
        print("Exiting - Joint Trajectory Action Complete")
        return 0
    
def listener():
    print("Initializing node... ")
    rospy.init_node('ekuri_ik_trajectory_client', anonymous=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("subscribing...")
    rospy.Subscriber("end_effector_command_solution", JointCommand, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()