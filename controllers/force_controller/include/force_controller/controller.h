#ifndef CONTROLLER_
#define CONTROLLER_

// ROS System
#include <ros/ros.h>

// ROS Message Types
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <force_controller/force_error_constantsConfig.h>
#include "/home/vmrguser/ros/indigo/baxter_ws/devel/include/force_controller/force_error_constantsConfig.h"

// Baxter Message Types
#include <baxter_core_msgs/JointCommand.h>  // To command the joints 
#include <baxter_core_msgs/EndpointState.h> // To read the wrench at the endpoint 
#include <baxter_core_msgs/SEAJointState.h> // To read the gravitation compensation data

// Eigen Libs
#include <Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>

// Force Control and Kinematics
#include <force_controller/forceControl.h>
#include <force_controller/kinematics.h>

// STD Libs
#include <iostream>
#include <fstream>
#include <string>
using std::string; 

#define PI 3.141592654

namespace force_controller

{
  // Arm Parameters
  static const int LEFT = 0, RIGHT = 1;

  // These are the parameters that conform a 2nd order least squares error approximation to a function composed by Baxter Joint Angles as the independent variable and Joint Torques as a dependent variable. These parameters are then used to model ax^2 + bx^1 + cx^0 for each of the 7 joints, for both arms giving rise to a 2x7x3 structure.
  static const double COEFF[2][7][3] = {
    // Left Arm
    { {0.04528400,  0.412655, -0.102458},     // S0
      {-13.645011, 15.076427, -2.089347},     // S1
      {9.60591200, 46.063440, 53.876335},     // E0
      {0.17552900, -0.174395, -1.880203},     // E1
      {2.78838700, -3.276785,  1.355042},     // W0
      {-0.2248960,  0.726712, -0.299096},     // W1
      {-0.2951670,  1.183197, -0.832453} },   // W2
    { // Right Arm
      {-0.8811330,  0.447500,  0.108815},     // S0
      {24.7798690,-44.860449, 17.881954},     // S1
      {-4.5111360, 22.012300,-24.573247},     // E0
      { 0.1717400, -0.207249, -0.946106},     // E1
      {-2.5302830, -3.267953, -0.817067},     // W0
      {-0.5545530,  1.847279, -1.338913},     // W1
      { 0.1445150,  0.506907,  0.507274} }	  // W2
  };

  class controller
  {
  public:
   
    // Constructor
  	controller(ros::NodeHandle node): node_handle_(node)
	  {
      n_ = 0;		m_ = 0;	no_ = 0;
      kf0=0.5; kf1=kf2=km0=km1=km2=0.0;
      int nj;
      double gain;
      double oneDeg = PI/180;
      double force_error_threshold=1;  

      /*** Get Parameter Values ***/
      // Get Parameter Values from the parameter server. Set in the roslaunch file or by hand.

      // Position Controller precision and filtering
      node_handle_.param<double>("joint_precision", tolerance_, oneDeg);
      node_handle_.param<double>("filter", alpha_, 0.9950);  //original 0.987512

      // Force Controller Error Tolerance
      node_handle_.param<double>("error_threshold",force_error_threshold_,force_error_threshold);

      // Strings
      node_handle_.param<std::string>("side", side_, "right");
      node_handle_.param<std::string>("tip_name", tip_name_, "right_gripper");

      // Gains
      node_handle_.param<double>("gain_force", gain, 0.0005); // Orig value: 0.00005)
      // gF_ = Eigen::Vector3d::Constant(gain);
      gF_ << kf0, kf1, kf2; 

      node_handle_.param<double>("gain_moment", gain, 0.0000035);
      gM_ = Eigen::Vector3d::Constant(gain);

      // Other vectors
      cur_data_ = Eigen::VectorXd::Zero(6);
      error_    = Eigen::VectorXd::Zero(6);

      // State
      exe_ = false;	jo_ready_ = false;

      /*** Publisher, subscriber and Service Advertisement ***/
      // 1. Subscribe to get current joint angle positions 
      joints_sub_ = root_handle_.subscribe<baxter_core_msgs::SEAJointState>("/robot/limb/" + side_ + "/gravity_compensation_torques", 1, &controller::updateJoints, this);

      // 2. Advertise a service to activate the control basis force/moment controllers
      ctrl_server_ = root_handle_.advertiseService("/" + side_ + "/force_controller", &controller::execute, this); // Creates service.

      // 3. Publisher to Joint 
      joint_cmd_pub_ = node_handle_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + side_ + "/joint_command", 20, false);
      ros::Duration(3.0).sleep();
      ROS_INFO("Initial Pose initialized for %s arm, tolerance = %f", side_.c_str(), tolerance_);	

      // Subscribe to control basis force/moment controller
      wrench_sub_ = root_handle_.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/" + side_ + "/endpoint_state", 1, &controller::getWrenchEndpoint, this);
      
      // Create the kinematic chain/model through kdl from base to right/left gripper
	    kine_model_ = Kinematics::create(tip_name_, nj);

      // Clear torque and gravitational torque vectors.
      torque_.clear();
      tg_.clear();

      // Confirm 7 DoF
      if(nj == 7)
        {
          fillJointNames();
          init_ = true;
        }

      // Print successful exit
      ros::Duration(1.0).sleep();
      ROS_INFO("Force controller on baxter's %s arm is ready", side_.c_str());
	  }

    // Destructor
    ~controller() { }

    // Public Methods
	  inline bool start() { return init_; }
    /*** Dynamic Reconfigure Callback ***/
    // void callback(force_error_constants::force_error_constantsConfig &config, uint32_t level); // placed it as global in .cpp
    

  private:

	  void fillJointNames();
	  Eigen::VectorXd getTorqueOffset();
	  sensor_msgs::JointState fill(Eigen::VectorXd dq);
	  std::vector<double> toVector(const geometry_msgs::Vector3& d);
    
    /*** ROS Updates for subscribers  and Parameters ***/
	  void updateJoints(const baxter_core_msgs::SEAJointStateConstPtr& state);
	  void updateGains(std::vector<geometry_msgs::Vector3> gain, std::vector<std::string> type);
    void getWrenchEndpoint(const baxter_core_msgs::EndpointStateConstPtr& state); // Eigen::Vector3d getWrenchEndpoint(....
    
    /*** Force Control and Null Space Methods ***/
    // double computeError(...) // inline method below.
    Eigen::Vector3d getDesiredForce(string type);
	  bool JacobianProduct(std::string type, Eigen::VectorXd& update);
	  bool NullSpaceProjection(std::vector<Eigen::VectorXd> updates, sensor_msgs::JointState& dq);
	  bool computePrimitiveController(std::vector<Eigen::VectorXd>& update, std::string type, geometry_msgs::Vector3 desired, std::vector<double>& e);

    // Controllers
    bool isMoveFinish(bool& result);                                           // Used by position control 
    void position_controller(sensor_msgs::JointState qd, ros::Time);           // Position Controller
    bool execute(forceControl::Request &req, forceControl::Response &res);     // Force Controller

    /*** Inline Methods ***/
	  inline void ini()
	  {
      std::ostringstream num2;
      num2 << "s" << m_ << "__" << "Joints_" << side_ << "_sim.txt";
      std::string title2 = num2.str();
      char * name2 = new char [title2.length()+1];
      std::strcpy (name2, title2.c_str());

      save_.open(name2, std::ios::out);
      exe_ = true;
	  }

	  inline void fin()
	  {
      exe_=false;
      if(save_.is_open())
        save_.close();

      joints_sub_.shutdown();
	  }


    //*******************************************************************************************
    // computeError(...)
    // Simply computes the difference between the desired amount and the actual amount.
    // type: force or moment
    // xt is the current data in force/moment
    // xd is the desired data 
    //*******************************************************************************************
	  inline double computeError(std::string type, Eigen::Vector3d xt, Eigen::Vector3d xd)
	  {
      double mag;
      int    offset = 0;
      if(type == "moment") offset = 3;

      // Compute the error between current and desired quantities, save in private member
      error_ = Eigen::VectorXd::Zero(6);
      for(unsigned int i=0; i<3; i++)
        error_(i+offset) = xt(i) - xd(i);

      // Also compute the norm. Useful to check if  error decreases over time. 
      mag = error_.norm();

      return mag;
	  }

    //*******************************************************************************************
    // Inlined Initialization function
    // Get's class' starting time. 
    // Sets a counter 
    //*******************************************************************************************
	  inline void initialize()
	  {
      to_ = ros::Time::now();
      m_ = 0;
	  }    

    // Node Handles
	  ros::NodeHandle node_handle_, root_handle_;          // Node handle is used with parameters and root_handle is used with services/publications/subscriptions

    // Publishers, subscribers, and services.
    // ros::Subscriber endPoint_sub_;                    // Not used currently.
	  ros::Subscriber joints_sub_;                         // Subscription to get joint angles. 
    ros::Publisher joint_cmd_pub_;                       // Publication to command joints 
	  ros::ServiceServer ctrl_server_;                     // Used to advertise the control basis service. 
    ros::Subscriber wrench_sub_;                         // Used to subscribe to the wrench endpoint. 

    // Kinematics model pointers.
	  Kinematics::Ptr kine_model_;
	 
    // Baxter arm strings
	  std::string side_, tip_name_;
	  std::vector<std::string> joints_names_;

    // Force Controller Vars
	  Eigen::Vector3d gF_, gM_;
	  Eigen::VectorXd error_;
    Eigen::Vector3d des_torque_;
    Eigen::VectorXd cur_data_;
	  std::vector<double> j_t_1_, tm_t_1_, tg_t_1_;
	  std::vector<std::vector<double> > joints_, torque_, tg_;

    // Position Controller Vars (used with controller::position_controller
	  baxter_core_msgs::JointCommand qgoal_;
	  std::vector<double> goal_, qd_, qe_; // Comes from position_controller/include/initialPose.h. 
                                         // joints_ was removed from here and instead we used the native std::vector<std::vector<double> > joints_ always using index[0] instead.

    // Position Controller Tolerance Parameters
	  double tolerance_, max_error_, alpha_;    

    // Force Controller Tolerance Parameters
    double force_error_threshold_;
    double kf0, kf1, kf2, km0, km1, km2; // gain constants. used with dynamic_reconfigure

    // Status Boolean Flags
	  bool init_, exe_, jo_ready_;

    // Counters
	  int n_, no_, m_, points_;

    // File Streams
	  std::ofstream save_;

    // Time
	  ros::Time to_;
  };
}
#endif /* REPLAY_ */
