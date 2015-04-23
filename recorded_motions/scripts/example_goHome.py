#!/usr/bin/env python

# Makes both the right and the left arm go home from which ever position they are at.
#import pdb 
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from recorded_motions import GoHome as gh

if __name__ == '__main__':
    try: 
        # If you want to debug, uncomment next line and import pdb
        #pdb.set_trace()

        # Init the ROS node
        rospy.init_node("go_home")        

        # Enable the robot's arms
        print("Getting robot state...")
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state=rs.state().enabled
        print("Enabling robot...")
        rs.enable()

        gh.GoHome()
    except:
        rospy.loginfo("GoHome node terminated...")
        rs.disable()
    
