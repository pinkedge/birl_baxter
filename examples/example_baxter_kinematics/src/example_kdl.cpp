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
Eigen::MatrixXd g_J; // Jacobian from (28) joint angles onto chosen target frame, w/rt chosen reference frame (nominally, pelvis)
Eigen::VectorXd g_x; // Cartesian Coordinates: x,y,z,r,p,y (forward kinematics answer)
Eigen::VectorXd g_q; // Joint Angles: Needed to compute forward kinematics and jacobian

// Jacobian solver object
// Automatically reads joint angles and produces cartesian output
TaskVariableXYZRPY *fwd_kin_solver_;
    
int main(int argc, char **argv)
{
  // Setup ROS
  ros::init(argc, argv, "task_variable_example");
  ros::NodeHandle nh;

  // KDL Solver Setup: used to parse robot_model tree, which should be made available in the ROS parameter server.
  TaskVariableSolver TemporaryHelper;
  TemporaryHelper.Init();
  
  // Jacobian Solver Allocation and Initialization
  // Create a ForwardMap and Jacobian object dynamically based on the size of the tree (depends on the number of joints).
  fwd_kin_solver_ = new TaskVariableXYZRPY(TemporaryHelper.my_tree);
  
  // Specify exactly from which joint to which joint do we want to solve for
  fwd_kin_solver_->Init(std::tuple<std::string, std::string>("pelvis","right_grasp_frame"));
  ROS_INFO("initialized solver"); 
  // done with solver set-up;
  
  // Initialize size and initialize matrices and vectors to zeros. (may not need to do this)
  g_q = Eigen::MatrixXd::Zero(28, 1);   // All joint angles initialized to zero
  g_x = Eigen::MatrixXd::Zero(6, 1);    // Cartesian Coordinates initialized to zero
  g_J = Eigen::MatrixXd::Zero(6, 9);    // 6x9 Jacobian from pelvis to right grasp frame initialized to zero
 
  // TaskVariableXYZRPY::Solve
  // 1. The FKkin object construct the Jacobian based on the input value of the joint angles.
  // 2. The Jacobian matrix for those values is placed in g_J.
  // 3. Finally, the Cartesian result is written to g_x. 
  
  // Manual Joint Angle Test
  // For this code, change one joint-angle value at-a-time. 
  // Then note how the value of the Jacobian changes. 
  // For example, let's change the value of joint angle 23 to lower the right arm from the shoulder joint 23.
  // g_q23->1.1->1.075->1.050... X should not change, the other parameters yes. 
  // cart
  //g_q(23)=1.100;      //  0.08191 -0.732582 -0.221134 -0.877445 -0.781693   2.61197                                      
  g_q(23)=1.075;        //  0.08191 -0.750847 -0.209566 -0.847426 -0.768835   2.59095 
  //g_q(23)=1.050;      //  0.08191 -0.768818 -0.197544 -0.818156 -0.755539   2.57074
  //g_q(23)=1.025;      //  0.08191 -0.786483 -0.185078 -0.789621 -0.741825   2.55132
  //g_q(23)=1.000;      //  0.08191 -0.803830 -0.172173 -0.761805 -0.727716   2.53267
  //g_q(23)=0.9900;       //  0.08191 -0.810678 -0.166891 -0.750875 -0.721967   2.52542
  fwd_kin_solver_->Solve(g_q.data(), g_J, g_x);         // fwd_kin_solver works with KDL. But we are passing Eigen vectors to it. 
                                                        // So, we need to use .data() to convert from Eigen type to compatible kdl argument
  //x_ = vlads_pose_hand_grab_wrt_pelvis_.head(3);      // can strip off a 3x1 vector to get origin of output frame as a point.
  
  // Output Data
  // Eigen objects are nicely formatted if use cout
  std::cout << "x = :";
  std::cout << g_x.transpose() << std::endl;    // output is x,y,z,R,P,Y, 
  std::cout << "J: "<< std::endl;               // Jacobian refers to joints numbered sequentially from reference frame to target frame
                                                // e.g., for reference = pelvis and target = right-hand grasp frame, uses joints 0,1,2,22,23,24,25,26,27
                                                // so J is 6x9
  std::cout << g_J << std::endl;
  }