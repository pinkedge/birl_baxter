#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#-----------------------------------------
# Overview:
#-----------------------------------------
# In this program we implement a numerical approach to inverse kinematics by using the Jacobian manipulator.
# In addition, we also make use of redundancy and the null space to achieve secondary goals in our solution.
# So, a reference cartesian location is given to the program. The error is computed and converted into a 
# joint angle perturbation that is added to the current joint angles. The error direction moves the joint angles
# towards the goal position. Further, we create a secondary goal, there are many possibilities:
# 	i) Move a single joint angle in a particular direction to avoid reaching a limit, or 
# 	ii) Move all joints towards the center of their joint motion range. This can be done by computing those center 
# 	positions and subtracting the current joint angles. This gives an error measure in joint angles.
# 
# About the Jabocian:
# The use of the Jaobian is not always evident. It works best with certain size perturbations. If the perturbations are
# too large the motions are uncontrolled. If the motions are too small, one may never reach the desired goal.
#
# About the null space and projections:
# After calculating the null space N, we will obtain a non-square matrix. This is in-effect a set of column vectors of joint
# angles whose linear combination will always result (when multiplied by the Jacobian) in a zero vector. 
# This non-square matrix can be turned into a projection matrix by multiplying the the null space times its inverse 
# (since this is not a square matrix, we multiply times the pseudo-inverse). The projection matrix can then be used to
# project any other joint-angle column vector into the null-space. This joint-angle vector actually represents dq, a
# joint angle velocity or perturbation. It is best represented by an error measure. If you have a desired joint angle state
# then subtract the current joint angle state to compute an error. This error should then be scaled to a size that works 
# well with the jacobian (otherwise it will result in an unstable motion). It is important to explicitly check to make sure
# the update is small. If it is not, it should not be used to move the joints.
#
# TODO:
# We have not yet checked for singularity conditions in the Jacobian. This can be done by studying the size of the determinant.
# If we are reaching a singularity condition, a recovery behavior should be effected. The motion path of the robot should be modified.
# 
# Requirements:
# baxter_PyKDL is needed and found at: http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL
# Using the Jacobian to move the arm to desired pose. 

#-----------------------------------------
# Imports
#-----------------------------------------
# To enable debug, uncomment the next line.
import pdb
import os                                          # used to clear the screen
import math
from numpy import *
from numpy.linalg import *
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from rbx1_nav.transform_utils import quat_to_angle # Convert quaternions to euler

#-----------------------------------------
# Local Methods
#-----------------------------------------
def enable_Baxter():
    # Enable the robot's arms
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state=rs.state().enabled
        
    print("Enabling robot...")
    rs.enable() 

    return rs

def shutdown():
    rospy.loginfo("Node has been terminated. Closing gracefully.")
    rospy.sleep(10)

def main():

    # If you want to debug, uncomment the next line.
    # pdb.set_trace()

    # Initialize node
    rospy.init_node('example_kdl')

    # Call routine to enable the robot
    rs=enable_Baxter()
    
    ## Params
    # Set a ROS Rate for the while loop at 1Hz
    loop=rospy.Rate(5) 

    # Set gain for multiplying error
    K=0.1

    # Create Limb Objects
    rLimb=baxter_interface.Limb('right')
    #lLimb=baxter_interface.Limb('left')

    # Create Kinematic Objects
    rKin = baxter_kinematics('right')
    #lKin = baxter_kinematics('left')

    # Get Joint names 
    jNamesR_ordrered=rLimb.joint_names()
    jNamesR=['right_s0','right_s1','right_w0','right_w1','right_w2','right_e0','right_e1']

    # Set Vector for mid-range of joint angles
    upper_limits=[0.890, 1.047, 3.028, 2.618, 3.059, 2.094, 3.059] # s0s1e0e1w0w1w2
    lower_limits=[-2.461, -2.147, -3.028, -0.052, -3.059, -1.571, -3.059]
    mid_range=[(upper_limits[i]-lower_limits[i])/2 for i in range(7)]
    q_ctr=[upper_limits[i]-mid_range[i] for i in range(7)]

    # Set a reference pose: [x,y,z,vx,vy,vz,w]. This one is near the home position of arm.
    # Home joint angle position for baxter_world_home.launch is approximately:
    # [0.58,-0.19,0.23,-0.11,0.99,0.01,0.02]
    ref_pose={'position':[0.70,-0.2, 0.23],'orientation':[-0.12,0.99,0.01,0.01]}
    ref_pos=matrix(ref_pose['position']).T
    ref_rot=matrix(ref_pose['orientation'])
    print 'The reference pos is: ' 
    print(ref_pos)

    # Right Arm
    #pdb.set_trace()
    while not rospy.is_shutdown():
        print 'example3_kdl for the right arm...'    

        # 1. Get the current angles and jacobian as numpy matrices
        rAngles=rLimb.joint_angles() # returns as s0s1w0w1w2e0e1
        rAnglesM=matrix(rAngles.values()).T
        jac=rKin.jacobian()

        # 2. Compute the forward Kinematics using the Jacobian: del_p=J(q)del_q
        curr_pos=rLimb.endpoint_pose()
        curr_pos=matrix(curr_pos['position']).T
        print 'The current position is: ' 
        print(curr_pos)

        # 3. Compute the error between reference and current positions
        dp_error=K*(ref_pos-curr_pos[0:3])

        # 4. Get a scalar distance (vector norm) to more easily interpret the error
        dp_norm=norm(dp_error)
        print 'the dpnorm is %f' %dp_norm
       
        # 5. Extract the translation Jacobian
        jTrans=jac[0:3,0:7] # when slicing go one dim past end 

        # 6. Compute the dq using dp_error
        dq=(pinv(jTrans))*dp_error

        # 7. Add this dq to current joint Angles
        ref_anglesM=dq+rAnglesM
        ref_angles=ref_anglesM.ravel().tolist()[0]  # change back to list.[0] b/c it returns nested list.
        ref_angles=dict(zip(jNamesR,ref_angles))
 
        # 8. Move the Limb
        rLimb.move_to_joint_positions(ref_angles) # need to check that indeces are in the right order

        # ---- Extra ---- Move to null space
        # import scipy
        # from scipy import linalg, matrix
        # def null(A, eps=1e-12):
        #     u, s, vh = scipy.linalg.svd(A)
        #     padding = max(0,np.shape(A)[1]-np.shape(s)[0])
        #     null_mask = np.concatenate(((s <= eps), np.ones((padding,),dtype=bool)),axis=0)
        #     null_space = scipy.compress(null_mask, vh, axis=0)
        #     return scipy.transpose(null_space)
        
        # # Apply    
        # A = matrix([[2,3,5],[-4,2,3]])
        # print A*null(A)


        # Set the loop speed
        loop.sleep()
        # Clear the screen for easier reading
        os.system('clear')

    # Set rospy to execute a shutdown function when exiting
    rospy.on_shutdown(shutdown)
    return rs                                                 


 
if __name__ == "__main__":
    try:
        main()
    except:
        rospy.loginfo("example_baxter_kins_right node terminated.")        
