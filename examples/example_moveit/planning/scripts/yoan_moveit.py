# -*- coding: utf-8 -*-

import rospy
import moveit_commander

from geometry_msgs.msg import *

rospy.init_node("baxter_moveit")

rospy.sleep(0.5)

group = moveit_commander.MoveGroupCommander("right_arm")

rospy.sleep(0.5)

p = Pose()
p.position.x = 0.133
p.position.y = -0.598
p.position.z = 0.012
p.orientation.x = -0.008
p.orientation.y = 0.707
p.orientation.z = -0.062
p.orientation.w = 0.704

group.set_pose_target(p)

group.go()
