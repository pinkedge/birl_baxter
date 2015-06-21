#!/usr/bin/env python  
import roslib
roslib.load_manifest('kinect_based_arm_tracking')
import rospy
import math
import tf
import geometry_msgs.msg
import math
import baxter_interface
from baxter_interface import CHECK_VERSION

import operator

#shoulder_a is the shoulder of the intested arm

mirror = True
counter = {'left':0, 'right':0}
data_set = {'left':[], 'right':[]}
times = 10


def vector_length(v):
    return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])

def get_unit_vector(v):
    length = vector_length(v)
    return (v[0]/length, v[1]/length, v[2]/length)

def angle_query(v1, v2):
    p = v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]
    m1 = vector_length(v1)
    m2 = vector_length(v2)
    cos = p/(m1*m2)
    return math.acos(cos)


def vector_cross_product(a, b):
    r1 = a[1]*b[2]-b[1]*a[2]
    r2 = a[2]*b[0]-b[2]*a[0]
    r3 = a[0]*b[1]-b[0]*a[1]
    return (r1, r2, r3)

def tf_2_angles(shoulder, shoulder_x, elbow, hand,  torso, head, base, listener, limb_name):
    try:
        (b_2_s, trash) = listener.lookupTransform(base, shoulder, rospy.Time(0))
        (b_2_s_x, trash) = listener.lookupTransform(base, shoulder_x, rospy.Time(0))
        (b_2_e, trash) = listener.lookupTransform(base, elbow, rospy.Time(0))
        (b_2_hand, trash) = listener.lookupTransform(base, hand, rospy.Time(0))
        (b_2_t, trash) = listener.lookupTransform(base, torso, rospy.Time(0))
        (b_2_head, trash) = listener.lookupTransform(base, head, rospy.Time(0))
        
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return {}

    b_2_s = list(b_2_s)
    b_2_s_x = list(b_2_s_x)
    b_2_e = list(b_2_e)
    b_2_hand = list(b_2_hand)
    b_2_t = list(b_2_t)
    b_2_head = list(b_2_head)
    
    
    
    s_2_s_x = map(operator.sub, b_2_s_x, b_2_s)
    s_2_t = map(operator.sub, b_2_t, b_2_s)
    s_2_e = map(operator.sub, b_2_e, b_2_s)
    e_2_hand = map(operator.sub, b_2_hand, b_2_e)
    
    head_2_t = map(operator.sub, b_2_t, b_2_head)
    
    my_v2 = head_2_t
    
    #print 'v2:', my_v2
    
    my_v3 = vector_cross_product(s_2_e, my_v2)
    
    #print 'v3:', my_v3
    
    
    my_theta_1 = angle_query(my_v3, s_2_s_x)
    
    
    
    #print 'my_theta_1:',my_theta_1/math.pi*180
    if limb_name == 'right':
        radius_s0 = math.pi/4-my_theta_1
    else:
        radius_s0 = math.pi*3/4-my_theta_1
        
        
    if mirror:
        if limb_name == 'left': 
            radius_s0 = -radius_s0+math.pi/2
        else:
            radius_s0 = -radius_s0-math.pi/2
    
    
    my_theta_2 = angle_query(my_v2, s_2_e)
    radius_s1 = math.pi/2-my_theta_2
    
    
    my_v4 = vector_cross_product(s_2_e, e_2_hand)
    
    #print "v4:", my_v4
    
    my_theta_3 = angle_query(my_v3,my_v4)
    
    
    radius_e0 = my_theta_3
    if limb_name == 'left':
        radius_e0 = -radius_e0
    
    my_theta_4 = angle_query(s_2_e,e_2_hand)
    radius_e1 = my_theta_4
    
    
    return {"s0":radius_s0, "s1":radius_s1, "e0":radius_e0, "e1":radius_e1}
    
def track_one_arm(shoulder, shoulder_x, elbow, hand, torso, head, base, listener, limb, limb_name):
    d = tf_2_angles(shoulder, shoulder_x, elbow, hand, torso, head, base, listener, limb_name)
        
    if d == {}:
        return
        
    radius_s0 = d["s0"]
    radius_s1 = d["s1"]
    radius_e0 = d["e0"]
    radius_e1 = d["e1"]
        
    degree = radius_s0/math.pi*180
    #print "s0 degree:",degree
        
    degree = radius_s1/math.pi*180
    #print "s1 degree:",degree
        
    degree = radius_e0/math.pi*180
    #print "e0 degree:",degree
        
    degree = radius_e1/math.pi*180
    #print "e1 degree:",degree

        
        
    
    
    
    counter[limb_name] = counter[limb_name]+1
    data_set[limb_name].append([radius_s0,
                                radius_s1,
                                radius_e0,
                                radius_e1])
    
    if counter[limb_name] == times:
        command = [0.0, 0.0, 0.0, 0.0]
        for i in range(0, 4):
            sum = 0
            for j in range(0, times):
                sum = sum+data_set[limb_name][j][i]
            command[i] = sum/times
        angles = limb.joint_angles()
        angles[limb_name+'_s0'] = command[0]
        angles[limb_name+'_s1'] = command[1]
        angles[limb_name+'_e0'] = command[2]
        angles[limb_name+'_e1'] = command[3]
        
        counter[limb_name] = 0
        data_set[limb_name] = []
        
        
        limb.set_joint_positions(angles)

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()



   
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    
    left_limb = baxter_interface.limb.Limb("left")
    right_limb = baxter_interface.limb.Limb("right")
    
    
    
    rospy.sleep(5)

    rate = rospy.Rate(51.0)
    
    for_my_left_shoulder = 'right_shoulder_1'
    for_my_left_shoulder_x = 'left_shoulder_1'
    for_my_left_elbow = 'right_elbow_1'
    for_my_left_hand = 'right_hand_1'
    
    for_my_right_shoulder = 'left_shoulder_1'
    for_my_right_shoulder_x = 'right_shoulder_1'
    for_my_right_elbow = 'left_elbow_1'
    for_my_right_hand = 'left_hand_1'
    
    head = 'head_1'
    torso = 'torso_1'
    base = 'camera_depth_frame'
    
    
    
    
    while not rospy.is_shutdown():
        limb_1 = left_limb
        limb_1_name = 'left'
        limb_2 = right_limb
        limb_2_name = 'right'
        
        if mirror == True:
            limb_1 = right_limb
            limb_1_name = 'right'
            limb_2 = left_limb
            limb_2_name = 'left'
        
      
        track_one_arm(for_my_left_shoulder
                        ,for_my_left_shoulder_x
                        ,for_my_left_elbow
                        ,for_my_left_hand
                        ,torso
                        ,head
                        ,base
                        ,listener
                        ,limb_1
                        ,limb_1_name)
        track_one_arm(for_my_right_shoulder
                        ,for_my_right_shoulder_x
                        ,for_my_right_elbow
                        ,for_my_right_hand
                        ,torso
                        ,head
                        ,base
                        ,listener
                        ,limb_2
                        ,limb_2_name)

                        
        
        
        
        rate.sleep()
