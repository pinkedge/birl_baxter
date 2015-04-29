#include <ros/ros.h>
#include <task_variables/library.h>     // KDL interface 

#include <Eigen/Eigen>                  // Linear Algebra
#include <Eigen/Dense>                  // Equation solvers
#include <Eigen/Core>
#include <Eigen/LU>                     // Linear Algebra - Lower and Upper decomposition

#include <atlas_msgs/AtlasState.h>      // Accessor for Atlas Joint Values

// General Libraries
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>
#include <stdio.h>

// Global Variables (Not necessary to create them here...)
// They are Matrix and Vectors from the Eigen Class
//double g_atlasState[NJoints];
Eigen::MatrixXd g_J;        // Jacobian from (28) joint angles onto chosen target frame, w/rt chosen reference frame (nominally, pelvis)
Eigen::VectorXd g_pose;     // Cartesian Coordinates: x,y,z,r,p,y (forward kinematics answer)
Eigen::VectorXd g_q;        // Joint Angles: Needed to compute forward kinematics and jacobian

// Jacobian solver object
// Automatically reads joint angles and produces cartesian output
TaskVariableXYZRPY *fwd_kin_solver_;

// Callback to subscribe to joint state messages; 
// copy results to global array for access by main()
void getJointStatesCB(const atlas_msgs::AtlasState& js) {
    for (unsigned int i = 0; i < NJoints; i++) {
        g_q(i) = js.position[i]; //copy to global var
    }
}

int main(int argc, char **argv) 
{
    // Setup ROS
    ros::init(argc, argv, "task_variable_example");
    ros::NodeHandle nh;

    // some useful variables, Eigen style:
    Eigen::Vector3d p_target;   // position vector, x,y,z, of target frame's origin w/rt named reference frame
    Eigen::Matrix3d R_target;   // orientation of target frame, expressed as a 3x3 rotation matrix
    
    ros::Rate timer(2);         //timer to run at 2 Hz
    
    // KDL Solver Setup: used to parse robot_model tree, which should be made available in the ROS parameter server.
    TaskVariableSolver TemporaryHelper;
    TemporaryHelper.Init();
    
    // Jacobian Solver Allocation and Initialization
    // Create a Jacobian object dynamically based on the size of the tree (depends on the number of joints).
    fwd_kin_solver_ = new TaskVariableXYZRPY(TemporaryHelper.my_tree);
    
    // Specify exactly from which joint to which joint do we want to solve for
    fwd_kin_solver_->Init(std::tuple<std::string, std::string>("pelvis", "right_grasp_frame"));
    ROS_INFO("initialized solver");
    // done with solver set-up;
    
    // Initialize size and initialize matrices and vectors to zeros. (may not need to do this)
    g_q                 = Eigen::MatrixXd::Zero(28, 1);     // All joint angles initialized to zero
    g_pose              = Eigen::MatrixXd::Zero(6, 1);      // Cartesian Coordinates initialized to zero
    g_J                 = Eigen::MatrixXd::Zero(6, 9);      // 6x9 Jacobian from pelvis to right grasp frame initialized to zero
    Eigen::MatrixXd J   = Eigen::MatrixXd::Zero(6, 6);      // Just consider the angles in the arm
    ROS_INFO("setting up subscriber to atlas state");
    
    // Subscribe to AtlasState and then copy joint angles to local variable
    ros::Subscriber sub = nh.subscribe("/atlas/atlas_state", 1, getJointStatesCB);
    ros::spinOnce();            // Callback process incoming messages one time

    // Solve for the Jacobian and print to screen
    while (ros::ok()) {
      timer.sleep();    // Runs while loop at rate established by ros::Rate timer. 
      ros::spinOnce();  // Update joint angles. 

      // TaskVariableXYZRPY::Solve
      // 1. The FKkin object construct the Jacobian based on the input value of the joint angles.
      // 2. The Jacobian matrix for those values is placed in g_J.
      // 3. Finally, the Cartesian result is written to g_x.
      fwd_kin_solver_->Solve(g_q.data(), g_J, g_pose);    // fwd_kin_solver works with KDL. But we are passing Eigen vectors to it. 
                                                          // So, we need to use .data() to convert from Eigen type to compatible kdl argument

      // Extract position and Rotation Matrix. 
      p_target = g_pose.head(3);      // Can strip off a 3x1 vector to get origin of output frame as a point.

      // Compute Rotation Matrix. 
      R_target = Eigen::AngleAxisd(g_pose(5), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(g_pose(4), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(g_pose(3), Eigen::Vector3d::UnitX());

      // Eigen objects are nicely formatted if use cout
      std::cout <<"joint angles: "<<std::endl;
      std::cout <<g_q<<std::endl;

      std::cout << "target frame origin = ";
      std::cout << p_target.transpose() << std::endl; // output is x,y,z

      std::cout << "target frame orientation is: " << std::endl;
      std::cout << R_target << std::endl;

      std::cout << "J: " << std::endl;    // Jacobian refers to joints numbered sequentially from reference frame to target frame
                                          // e.g., for reference = pelvis and target = right-hand grasp frame, uses joints [0,1,2],[22,23,24,25,26,27]
                                          // so J is 6x9
      std::cout << g_J << std::endl;
    }

}
