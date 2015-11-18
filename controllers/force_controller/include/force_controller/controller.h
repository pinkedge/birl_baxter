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

// Forcing Core Files (http://processors.wiki.ti.com/index.php/Multithreaded_Debugging_Made_Easier_by_Forcing_Core_Dumps)
//include <sys/types.h>
//include <signal.h>
//include <unistd.h>

// STD Libs
#include <iostream>
#include <fstream>
#include <string>
#include <deque>
using std::string; 

//-----------------------------------------------------------------------------------------------------------------------------
// Programs design parameters
//-----------------------------------------------------------------------------------------------------------------------------
/*** Flags for ROS Communication Objects ***/
#define JOINTS_SUB_F 1 // Subscribes to /gravity_compensation_torques to get joints, velocities, torques, and gravity compensations
#define WRENCH_SUB_F 1 // Subscribes to /endpoint_state to get the endpoint wrench
#define JOINTS_PUB_F 1 // Publishes to /joint_command to move the arm to a reference set point.
#define FILT_W_PUB_F 1 // Publishes a filtered wrench value 
#define CTRBAS_SRV_F 1 // Publishes the control basis service server. When a client call is sent, force_control begins. 
#define DYN_RECONF_F 1

/*** Inner Control Loop ***/ 
#define JNTPOS_TORQUE_CONTROLLER 1     // If true, set point is joint angles, otherwise joint torques.

/*** Time  Rates ***/
#define ROS_RATE     1000              // These rates are very important. ROS_RATE controls while loop timing. 
#define TIME_OUT     1.5               // This timeout determines how long the inner pos_Ctrl will run. Very important. 
                                       // If too long and robot is in contact with surface, forces will rise dangerously.
                                       // We want to add delta joint angles to the latest joint position. To do this, we act as fast as possible, or else we'll be adding dq's to old joint valus. Also, joint_command works best at a fast rate.

/*** MATH ***/
#define PI 3.141592654
//-----------------------------------------------------------------------------------------------------------------------------

namespace force_controller
{
  // Arm Parameters
  static const int LEFT = 0, RIGHT = 1;

  // Proportional Gain Parameters for joint controller Const: (0.0050)
  double pg= 0; // 0.0050;
  double k_fp0=0.015, k_fp1=pg, k_fp2=pg, k_mp0=pg, k_mp1=pg, k_mp2=pg; // gain constants. used with dynamc_reconfigure
  

  // Derivative Gain Parameters for joint controller Const: Const: 0.0025
  double dg=0; // 0.0025;
  double k_fv0=dg, k_fv1=dg, k_fv2=dg, k_mv0=dg, k_mv1=dg, k_mv2=dg;

  bool force_error_constantsFlag = false;

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
      // Initialize counters and Flags
      n_ = 0;	m_ = 0;	no_ = 0, errorCtr_=1; error_norm_=0.0;

      // Local 
      int nj;
      double gain;
      // Default parameter Values
      double alpha  = 0.9950;           // Orig: 0.987512
      double oneDeg = PI/180;
      int    force_error_threshold=1;  

      /*** Get Parameter Values ***/
      // Get Parameter Values from the parameter server. Set in the roslaunch file or by hand.

      // Position Controller precision and filtering
      node_handle_.param<double>("joint_precision", tolerance_, oneDeg);
      node_handle_.param<double>("filter", alpha_, alpha);  

      // Force Controller Error Tolerance
      node_handle_.param<double>("error_threshold",force_error_threshold_,force_error_threshold);

      // Strings
      node_handle_.param<std::string>("side", side_, "right");
      node_handle_.param<std::string>("tip_name", tip_name_, "right_gripper");

      // Hack: currently we cannot guarantee the order in which spinner threads are called.
      // There are occassions in which getWrenchEndpoint is called before updateJointAngles, in this case, getTorqueOffset is called, which needs joints. A segfault is issued.
      std::vector<double> tmp;
      //for(int i=0; i<7; i++) tmp.push_back(0.0);
      tmp.resize(7);
      tmp[0]=0.07; tmp[1]=-0.9; tmp[2]=1.15; tmp[3]=1.92; tmp[4]=-0.6; tmp[5]=1.01; tmp[6]=0.48;

      // Right Arm
      if(strcmp(side_.c_str(),"right")) joints_.push_back(tmp);
      // Left arm
      else 
        {
          tmp[0]=0.0; tmp[1]=-0.9; tmp[2]=1.1; tmp[3]=1.92; tmp[4]=-0.65; tmp[5]=1.00; tmp[6]=-0.48;
          joints_.push_back(tmp);
        }
          
      // Proportional Gains
      node_handle_.param<double>("gain_force", gain, 0.0005); // Orig value: 0.00005)
      // gFp_ = Eigen::Vector3d::Constant(gain);
      gFp_ << k_fp0, k_fp1, k_fp2; 

      node_handle_.param<double>("gain_moment", gain, 0.0000035);
      gMp_ = Eigen::Vector3d::Constant(gain);

      // Derivative Gains
      gFv_ << k_fv0, k_fv1, k_fv2; 
      gMv_ << k_mv0, k_mv1, k_mv2; 

      // Other vectors
      setPoint_   = Eigen::VectorXd::Zero(3);
      cur_data_   = Eigen::VectorXd::Zero(6);  cur_data_f_ = Eigen::VectorXd::Zero(6);
      error_      = Eigen::VectorXd::Zero(6);  error_t_1   = Eigen::VectorXd::Zero(6);
      derror_     = Eigen::VectorXd::Zero(6);

      // State
      exe_ = false;	jo_ready_ = false;

      /***************************************************** Publisher, subscriber and Service Advertisement *******************************************************************/
      // Set all the flag values ros ros communication objects
      rosCommunicationCtr=0;
      joints_sub_flag         =JOINTS_SUB_F; 
      wrench_sub_flag         =WRENCH_SUB_F;
      joint_cmd_pub_flag      =JOINTS_PUB_F;
      filtered_wrench_pub_flag=FILT_W_PUB_F;
      ctrl_server_flag        =CTRBAS_SRV_F; 
      dynamic_reconfigure_flag=DYN_RECONF_F; 

      // 1. Subscription object to get current joint angle positions, velocities, joint torques, and gravitational compensation torques.
      if(joints_sub_flag)
        {
          joints_sub_ = root_handle_.subscribe<baxter_core_msgs::SEAJointState>("/robot/limb/" + side_ + "/gravity_compensation_torques", 1, &controller::getBaxterJointState, this);
          rosCommunicationCtr++;
        }

      // 2. Subscription object to get the wrench endpoint state. 
      if(wrench_sub_flag)
        {
          wrench_sub_ = root_handle_.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/" + side_ + "/endpoint_state", 1, &controller::getWrenchEndpoint, this);
          rosCommunicationCtr++;
        }

      // 3. Publication object to publish commanded joint positions throught the joint_command topic.
      if(joint_cmd_pub_flag)
        {
          joint_cmd_pub_ = node_handle_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + side_ + "/joint_command", 10, false); // May want to keep a small num of points
          ros::Duration(3.0).sleep(); ROS_INFO("Initial Pose initialized for %s arm, tolerance = %f", side_.c_str(), tolerance_);	
          rosCommunicationCtr++;
        }

      // 4. Publication object to publish filtered wrench information.
      if(filtered_wrench_pub_flag)
        {
          filtered_wrench_pub_ = node_handle_.advertise<baxter_core_msgs::EndpointState>("/robot/limb/" + side_ + "/filtered_wrench", 20, false);
          rosCommunicationCtr++;
        }

      // 5. Service Server Object. Runs the force_controller when a desired primitive controllers is selected along with the desired setpoint. 
      if(ctrl_server_flag)
        {
          ctrl_server_ = root_handle_.advertiseService("/" + side_ + "/force_controller", &controller::force_controller, this); 
          rosCommunicationCtr++;
        }
      
      // Create the kinematic chain/model through kdl from base to right/left gripper
	    kine_model_ = Kinematics::create(tip_name_, nj);

      // Clear torque and gravitational torque vectors.
      torque_.clear();
      tg_.clear();

      // If we have our 7DoF, then let's set the values of our joint names in joints_names_ as: s0s1e0e1w0w1w2. This is the order of /robot/limb/right/gravity_compensation_torques but not of /robot/limb/right/joint_commad
      if(nj == 7)
        {
          fillJointNames();
          init_ = true;
        }

      // Print successful exit
      ros::Duration(1.0).sleep();

      // Time Rates
      timeOut_        =TIME_OUT;
      while_loop_rate_=ROS_RATE;

      /*** Filtering ***/
      // Inner Control Loop
      jntPos_Torque_InnerCtrl_Flag_=JNTPOS_TORQUE_CONTROLLER;

        // Wrench Filtering
      wrenchFilteringFlag=1;
      initialFiltering=1;

      ROS_INFO("Force controller on baxter's %s arm is ready", side_.c_str());
	  }

    // Destructor
    ~controller() { }

    // Public Flags
    int dynamic_reconfigure_flag;

    // Public Methods
	  inline bool start() { return init_; }
    inline int get_rosCommunicationCtr() { return rosCommunicationCtr; }
    inline void rosCommunicationCtrUp() { rosCommunicationCtr++; }
    /*** Dynamic Reconfigure Callback ***/
    // void callback(force_error_constants::force_error_constantsConfig &config, uint32_t level); // placed it as global in .cpp
    

  private:

	  void fillJointNames();
	  Eigen::VectorXd         getTorqueOffset();
	  std::vector<double>     toVector(const geometry_msgs::Vector3& d);
	  sensor_msgs::JointState fill(Eigen::VectorXd dq);

    /*** ROS Updates for subscribers  and Parameters ***/
    void getWrenchEndpoint(const baxter_core_msgs::EndpointStateConstPtr& state);    // Eigen::Vector3d getWrenchEndpoint(....
	  void getBaxterJointState(const baxter_core_msgs::SEAJointStateConstPtr& state);  // Used to get joint positions, velocities, and efforts from Baxter. 

    void updateGains();                                                              // used to change gFp_ and gMp_ after params updated with rqt_reconfig
	  void updateGains(std::vector<geometry_msgs::Vector3> gain, std::vector<std::string> type);
    
    /*** Controllers ***/
    bool isMoveFinish(bool& result);                                                  // Used by position control to check if goal has been reached.
    bool position_controller(sensor_msgs::JointState qd, ros::Time);                  // Position Controller
    bool force_controller(forceControl::Request &req, forceControl::Response &res);   // Force Controller
    void torque_controller(Eigen::VectorXd delT, ros::Time t0);                       // Torque controller

    /*** Force Control Support Methods and Null Space Methods ***/
    // double computeError(...) // inline method below.
    Eigen::Vector3d extractWrench_Force_Moment(string type);
	  bool JacobianProduct(std::string type, Eigen::VectorXd& update);
	  bool JacobianErrorProduct(std::string type, Eigen::VectorXd& update); // changed the order of multiplication by gains to see if it improves controller.
	  bool NullSpaceProjection(std::vector<Eigen::VectorXd> updates, sensor_msgs::JointState& dq);
	  bool computePrimitiveController(std::vector<Eigen::VectorXd>& update, std::string type, Eigen::Vector3d setPoint, std::vector<double>& e);



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

      // Compute the error between cur and des  data, save in error_ private member
      error_ = Eigen::VectorXd::Zero(6);
      for(unsigned int i=0; i<3; i++)
        error_(i+offset) = (xd(i)-xt(i)); // -1 is to help us descend the gradient error. 
        
      // Compute the derivative error using a finite difference approximation. 
      // TODO: should try the symmetric difference quotient. (error+1-error_1)/2rate
      // TODO: Can filter the derror signal.
      int position_derror_flag=1;
      if(position_derror_flag)
        {
          for(unsigned int i=0; i<3; i++)     
            derror_(i+offset) = (error_(i+offset) - error_t_1(i+offset))*while_loop_rate_;          // Instead of dividing by time, multiply by the rate.
        }
      else
        {
          for(unsigned int i=0; i<3; i++)     
            derror_(i+offset) = (velocity_[0][i+offset] - velocity_[1][i+offset])*while_loop_rate_; // Instead of dividing by time, multiply by the rate.
        }

      if(errorCtr_==1)
        derror_=Eigen::VectorXd::Zero(6);
      
      // Save current error to error_t_1
      error_t_1 = error_;

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

    /***************************************************************************** Private Members **********************************************************/
    // Node Handles
	  ros::NodeHandle node_handle_, root_handle_;          // Node handle is used with parameters and root_handle is used with services/publications/subscriptions
                                                         // Designed to work with local and global namespaces. Currently not used consistently. 

    // Publishers, subscribers, and services.
	  ros::Subscriber    joints_sub_;                      // Subscription to get joint angles. 
    ros::Subscriber    wrench_sub_;                      // Used to subscribe to the wrench endpoint. 

    ros::Publisher     joint_cmd_pub_;                   // Publication to command joints 
    ros::Publisher     filtered_wrench_pub_;             // Publish a filtered wrench number

	  ros::ServiceServer ctrl_server_;                     // Used to advertise the control basis service. 

    // Publish/Subscribe/Service Flags and Counter. 
    // Given that we are using a multithreaded method, it's good to automatically generate a counter of how many things we publish/subscribe/service.
    int rosCommunicationCtr;

    int joints_sub_flag;
    int wrench_sub_flag;

    int joint_cmd_pub_flag;
    int filtered_wrench_pub_flag;

    int ctrl_server_flag;
    // int dynamic_reconfigure_flag;                       // Currently dynamic_reconfigure code sits outside the class in main, so this will be publc.


    // Kinematics model pointers.
	  Kinematics::Ptr kine_model_;
	 
    // Baxter arm strings
	  std::string side_, tip_name_;
	  std::vector<std::string> joints_names_;

    // Force Controller Vars
	  Eigen::Vector3d gFp_, gMp_; // Proportional gains
    Eigen::Vector3d gFv_, gMv_; // Derivative gains
	  Eigen::VectorXd error_, error_t_1, derror_;
    double error_norm_;
    Eigen::VectorXd setPoint_;
    Eigen::VectorXd cur_data_, cur_data_f_;
    std::deque<Eigen::VectorXd> wrenchVec, wrenchVecF; 
	  std::vector<double> j_t_1_, jv_t_1_, tm_t_1_, tg_t_1_; // Joints, velocity, torques, gravitational torques. 
	  std::vector<std::vector<double> > joints_, velocity_, torque_, tg_;

    // Position Controller Vars (used with controller::position_controller
	  baxter_core_msgs::JointCommand qgoal_;
	  std::vector<double> goal_, qd_, qe_; // Comes from position_controller/include/initialPose.h. 
                                         // joints_ was removed from here and instead we used the native std::vector<std::vector<double> > joints_ always using index[0] instead.

    // Position Controller Tolerance Parameters
	  double tolerance_, max_error_, alpha_;    

    // Force Controller Tolerance Parameters
    double force_error_threshold_;

    // Status Boolean Flags
	  bool init_, exe_, jo_ready_;

    // Counters
	  int n_, no_, m_, points_, errorCtr_;

    /*** Flags ***/
    // Inner Control Loop Flags
    int jntPos_Torque_InnerCtrl_Flag_;
    // Wrench Filtering Flags
    int wrenchFilteringFlag;  // in getWrenchEndpoint()
    int initialFiltering;

    // 


    // File Streams
	  std::ofstream save_;

    // Time
	  ros::Time to_;
    double timeOut_;
    int while_loop_rate_;

  };
}
#endif /* REPLAY_ */
