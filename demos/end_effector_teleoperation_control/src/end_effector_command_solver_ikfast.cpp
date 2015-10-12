#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IKREAL_TYPE IkReal // for IKFast 56,61

#include "baxter_right_arm_ikfast_solver.cpp"
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <string>

using namespace geometry_msgs;

ros::Publisher leftJointCommandPublisher;
ros::Publisher rightJointCommandPublisher;

void findIKSolutions(double _tx, double _ty, double _tz, double _qw, double _qx, double _qy, double _qz);

void callBack(const PoseStamped &target) {
    if (target.header.frame_id.find("left") != string::npos) {
        ROS_INFO_STREAM("left arm command recieved");
        leftJointCommandPublisher.publish(target);
    } else {
        rightJointCommandPublisher.publish(target);
        ROS_INFO_STREAM("right arm command recieved");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_command_solver_ikfast");
    ros::NodeHandle n;
    leftJointCommandPublisher = n.advertise<geometry_msgs::PoseStamped>("end_effector_left_end_command_position", 1);
    rightJointCommandPublisher = n.advertise<geometry_msgs::PoseStamped>("end_effector_right_end_command_position", 1);
    ros:: Subscriber s = n.subscribe("end_effector_command_position", 1, callBack);
    ROS_INFO_STREAM("end_effector_command_solver_ikfast subscribing...");
    ros::spin();
    return 0;
}
