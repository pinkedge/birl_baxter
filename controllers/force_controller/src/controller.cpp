#include <force_controller/controller.h>
// #include <dynamic_reconfigure/server.h>
// #include <force_controller/force_error_constantsConfig.h>

// TODO: in compute primitive controller compute jacobian once and then pass it to getWrenchEndpoint and to JacobianProduct. 
namespace force_controller
{
  //***********************************************************************************************************************************************
  // callback(...) for dynamic Reconfigure set as a global function. 
  // When the rqt_reconfigure gui is used and those parameters are changed, the config.param_name in this function will be updated. Then, these parameters need to be set to the private members of your code to record those changes. 
  //***********************************************************************************************************************************************
  void callback(force_error_constants::force_error_constantsConfig &config, uint32_t level)
  {
    // Print the updated values
    ROS_INFO("Reconfigure request: %f %f %f %f %f %f", 
             config.kf0,
             config.kf1,
             config.kf2,
             config.km0,
             config.km1,
             config.km2);
  
  // Save to the private data members. 
  // kf0=config.kf0;

  }
  //***********************************************************************************************************************************************
  // fillJointNames()
  // Given the string parameter for the right or left side of the arm, create a vector of joint names with an ordered set of joint angle elements. 
  //***********************************************************************************************************************************************
  void controller::fillJointNames()
  {
    joints_names_.clear();
    joints_names_.push_back(side_ + "_s0");
    joints_names_.push_back(side_ + "_s1");
    joints_names_.push_back(side_ + "_e0");
    joints_names_.push_back(side_ + "_e1");
    joints_names_.push_back(side_ + "_w0");
    joints_names_.push_back(side_ + "_w1");
    joints_names_.push_back(side_ + "_w2");

    // Resize other kinematic model variables: clear and resize. 
    joints_.clear();
    j_t_1_.clear();
    j_t_1_.resize(joints_names_.size());
    tm_t_1_.resize(joints_names_.size());
    tg_t_1_.resize(joints_names_.size());
  }

  // Convert a 3D geometry vector and convert it into a 3D std::vector 
  std::vector<double> controller::toVector(const geometry_msgs::Vector3& d)
  {
    std::vector<double> result;
    result.clear();
    result.push_back(d.x);
    result.push_back(d.y);
    result.push_back(d.z);
    return result;
  }

  // Takes a delta joint angle update and add it to the current angles.
  sensor_msgs::JointState controller::fill(Eigen::VectorXd dq)
  {
    // Create a JointState structure. Composed of string[] name and float64[] pos/vel/effort.
    sensor_msgs::JointState update;
    update.name = joints_names_;
    update.position.clear();

    // Add delta joint angle updates to current angles.
    for(unsigned int i=0; i<joints_names_.size(); i++)
      update.position.push_back(dq(i)+joints_[0][i]);

    // Clear other quantities. 
    update.velocity.clear();
    update.effort.clear();

    // Add current time stamp. 
    update.header.stamp = ros::Time::now();

    return update;
  }

  // This function records the torque offset 
  Eigen::VectorXd controller::getTorqueOffset()
  {
    // Create a 7 dimensional vector to hold torque offsets
    Eigen::VectorXd torqueOffset = Eigen::VectorXd::Zero(7);

    // Different offsets for right or left arms. 
    if(side_ == "left")
      {
        // Offset_left_ji = a_ji*x^2 + b_ji*x^1 + c_ji*x^0
        for(unsigned int i=0; i<7; i++)
          torqueOffset(i) = COEFF[LEFT][i][0]*joints_[0][i]*joints_[0][i] + COEFF[LEFT][i][1]*joints_[0][i] + COEFF[LEFT][i][2];
      }
    else if(side_ == "right")
      {
        // Offset_right_ji = a_ji*x^2 + b_ji*x^1 + c_ji*x^0
        for(unsigned int i=0; i<7; i++) 
          torqueOffset(i) = COEFF[RIGHT][i][0]*joints_[0][i]*joints_[0][i] + COEFF[RIGHT][i][1]*joints_[0][i] + COEFF[RIGHT][i][2];
      }
    else
      ROS_ERROR_STREAM_ONCE("The torque offset has not been set appropriately because the wrong arm name has been provided as: " << side_.c_str());

    return torqueOffset; // 7D vector of offsets
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // getDesiredForce()
  // Extracts either force or moment from wrench based on the input string. It assumes that a subscriber for the wrench data is being called and that the data is available.
  // Input: string for force or moment.
  // Returns 3D vector.
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
  Eigen::Vector3d controller::getDesiredForce(string type)
  {
    Eigen::Vector3d wrench = Eigen::Vector3d::Zero(); 

    // Split values into force or moment
    if(type == "force")
      {
        for(unsigned int i=0; i<3; i++)
          wrench(i) = cur_data_(i);
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          wrench(i) = cur_data_(i+3);
      }
    else
      ROS_ERROR("Type is incorrect");

    return wrench; 
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // 2 possible ways of getting the wrench endpoint:
  // Input: a baxter_core_msgs/EndPointState which contains a wrench with force/torque.
  // 1. Subscribe to the endpoint_state topic and get the wrench. 
  // 2. Get joint torques (and the gravitational torque) and then compute the endpoint wrench. Can us an offset (computed a priori) that tries to cancel the noise in Baxter's arms.  // Places result in private member cur_data_
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  void controller::getWrenchEndpoint(const baxter_core_msgs::EndpointStateConstPtr& state)
  {  
    // Local variables. Used in 2. 
    // Eigen::Vector3d result = Eigen::Vector3d::Zero();
    Eigen::VectorXd offset = getTorqueOffset(), realTorque, wrench; // Create 3 vectors: offset, realTorque, and wrench (force error). 
    Eigen::MatrixXd jacobian, JJt;

    // Assign the correct size for the vectors. 
    realTorque = Eigen::VectorXd::Zero(7);
    wrench     = Eigen::VectorXd::Zero(6);
    
    // Select method 1 or 2. 
    int endpointWrenchFlag = 1;
    if(endpointWrenchFlag) {
      
      // Get the force and torque
      cur_data_ << state->wrench.force.x,state->wrench.force.y,state->wrench.force.z,state->wrench.torque.x,state->wrench.torque.y,state->wrench.torque.z;
    }

    // Convert from joint torques to wrench.
    else {
      // Real Torque = sensed torque - gravitational torque - modeled offset torque. 
      for(unsigned int i=0; i<7; i++)
        realTorque(i) = torque_[0][i] - tg_[0][i]; //  - offset(i);

      // Print offset and real torques
      ROS_WARN_STREAM("Torque Offset is:\n--------------------\n" << offset     << "\n--------------------\n");
      ROS_WARN_STREAM("Real Torque is:  \n--------------------\n" << realTorque << "\n--------------------\n");

      // Get Jacobian (input: joint angles and joint names, output is matrix). 
      kine_model_->getJacobian(joints_[0], joints_names_, jacobian);

      // Preparing the Pseudo Inverse: 
      JJt = jacobian * jacobian.transpose();

      // Use the Jacobian Pseudo Inverse (Moore-Penrose Inverse) to compute the wrench.
      // Equation: (J'J)^-1*J' or J'*(JJ')^-1
      // Note that in our case T=J'e, so we need to use J' instead of just J in the above equation.
      // Also, the Pseudo Inverse tends to have stability problems around singularities. A large change in joing angles will be produced, even for a small motion.  
      // Singularities represent directions of motion that the manipulator cannot achieve. Particulary in a straight arm configuration, or in the wrist when W1 aligns both W0 and W2. We can check for near singularities if the determinant is close to zero. At singular positions the Pseudo Inverse the matrix is well behaved.
      cur_data_ = (JJt.inverse()*jacobian) * realTorque;
      ROS_INFO_STREAM("\n-------------------------------\nThe current wrench value is:\n-------------------------------\n" << wrench << "\n-------------------------------\n");
      // Use Eigen's least square approach: 
      // wrench = Jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(realTorque);
      for(unsigned int i=0; i<6; i++)
        {
          if(isnan(wrench(i)))
            {
              ROS_ERROR_STREAM("Failed to compute Jacobian Pseudo Inverse " << wrench << ", tor " << realTorque << ", jac" << jacobian << ", j " << joints_[0][0]);
              // return cur_data_;
            }
        }
    } // End if-else for two methds.   
  }

  //-----------------------------------------------------------------------------------------------------------------------------------------------
  // updateJoints
  // 
  // This callback function works in hand with the subscription call joints_sub_ using the topic /robot/limb/"side"/gravity_compensation_torques
  // Thsi topic allows us to obtain both the latest joint angle positions of the robot as well as current torque values. 
  // The type of the topic is: baxter_core_msgs::SEAJointState. It's strucutre which looks as follows:
  // ---
  //   std_msgs/Header header
  //   uint32 seq
  //   time stamp
  //   string frame_id
  //   string[] name
  //   ---
  //   float64[] commanded_position      
  //   float64[] commanded_velocity
  //   float64[] commanded_acceleration
  //   float64[] commanded_effort
  //   ---
  //   float64[] actual_position         // Current joint angle position in radians
  //   float64[] actual_velocity
  //   float64[] actual_effort           // This includes the inertial feed forward torques when applicable.
  //   float64[] gravity_model_effort    // This is the torque required to hold the arm against gravity returned by KDL if the arm was stationary.  
  //                                   // This does not include inertial feed forward torques (even when we have them) or any of the corrections (i.e. spring
  //   hysteresis, crosstalk, etc) we make to the KDL model.
  //   float64[] gravity_only
  //   float64[] hysteresis_model_effort
  //   float64[] crosstalk_model_effort
  //   float64 hystState
  //-----------------------------------------------------------------------------------------------------------------------------------------------
  void controller::updateJoints(const baxter_core_msgs::SEAJointStateConstPtr& state)
  {
    // Local variables
    ros::Time t = ros::Time::now();
    unsigned int i, k=0;

    // Create Joint, Torque, and Gravitation values and filtered values. 
    std::vector<double> jt, jtf;    // Joints and Filtered Joints
    std::vector<double> tt, tg;     // Torques and Filtered Torques
    std::vector<double> ttf, tgf;   // Gravitational Torques and Filtered equivalents.

    // Joint Angles
    jt.clear();		jtf.clear();
    jt.resize(joints_names_.size());
    jtf.resize(joints_names_.size());

    // Regular Torques
    tt.resize(joints_names_.size());
    ttf.resize(joints_names_.size());

    // Gravitational Torques
    tg.resize(joints_names_.size());
    tgf.resize(joints_names_.size());

    // Update the current joint values, actual effort, and gravity_model_effort values through the topic data.
    ROS_INFO_ONCE("Initializing joints");
    if(exe_ && save_.is_open())
      save_ << (t - to_).toSec() << "	"; // Convert time to seconds

    // For all of our 7 Joints
    while(k < joints_names_.size())
      {
        // And again for the 7 joints
        for(i=0; i<state->name.size(); i++)
          {            
            if(state->name[i] == joints_names_[k])
              {
                // Get current joint angles
                jt[k] = state->actual_position[i];

                // Get current joint torques
                tt[k] = state->actual_effort[i];

                // Get currend end-point gravitational torque 
                tg[k] = state->gravity_model_effort[i];

                // Save joint angle information
                if(exe_ && save_.is_open())		  
                  save_ << state->actual_position[i] << "	";

                k = k + 1;
                if(k == joints_names_.size())
                  break;
              }
          }
      }

    // Make sure previous values are set before performing IIR filtering. 
    if(!jo_ready_)
      {
        // Save current values into previous values
        for(unsigned int i=0; i<joints_names_.size(); i++)
          {
            ttf[i] = tt[i];
            tgf[i] = tg[i];

            // Create previous values
            tm_t_1_[i] = tt[i];
            tg_t_1_[i] = tg[i];
            j_t_1_[i] = jt[i];
          }

        // Save previous values before computing IIR Filtering
        torque_.push_back(tm_t_1_);
        torque_.push_back(tm_t_1_);
        
        tg_.push_back(tg_t_1_);
        tg_.push_back(tg_t_1_);
        
        joints_.push_back(jt);
        joints_.push_back(jt);
        
        jo_ready_ = true;
      }

    // IIR Low Pass Filter 
    for(unsigned int i=0; i<7; i++)
      {
        tgf[i] = 0.0784*tg_t_1_[i] + 1.5622*tg_[0][i] - 0.6413*tg_[1][i];
        ttf[i] = 0.0784*tm_t_1_[i] + 1.5622*torque_[0][i] - 0.6413*torque_[1][i];
        jtf[i] = 0.0784*j_t_1_[i] + 1.5622*joints_[0][i] - 0.6413*joints_[1][i];

        tm_t_1_[i] = tt[i];
        tg_t_1_[i] = tg[i];
        j_t_1_[i]  = jt[i];

        tg_[1][i]     = tg_[0][i];
        tg_[0][i]     = tgf[i];

        torque_[1][i] = torque_[0][i];
        torque_[0][i] = ttf[i];

        joints_[1][i] = joints_[0][i];
        joints_[0][i] = jtf[i];

      }

    // Save torque and gravitation torque info to file.
    if(exe_ && save_.is_open())
      {
        for(unsigned int j=0; j<joints_.size(); j++)
          save_ << tt[j] << "	";
        for(unsigned int j=0; j<joints_.size(); j++)
          save_ << tg[j] << "	";
        save_ << std::endl;
      }

    ROS_INFO_STREAM_ONCE("Joints updated for: " << ttf[0] << ", "<< ttf[1] << ", "<< ttf[2]);
  }

  // Used to update gains at run-time. 
  void controller::updateGains(std::vector<geometry_msgs::Vector3> gain, std::vector<std::string> type)
  {
    if(gain.size() != type.size())
      {
        ROS_WARN("Gain data does not math number of type controllers, using default gain");
        return;
      }
    for(unsigned int i=0; i<gain.size(); i++)
      {
        if(type[i] == "force")
          gF_ = Eigen::Vector3d(gain[i].x, gain[i].y, gain[i].z);
        else if(type[i] == "moment")
          gM_ = Eigen::Vector3d(gain[i].x, gain[i].y, gain[i].z);
        else
          ROS_WARN("Could not recognize type of controller, using default gain value");
      }
  }

/*********************************************** JacobianProduct ***********************************************************
 ** The Jacobian product can be computed using the pseudoinverse J#, or the Jacobian Transpose Jt in the case of position control
 ** and transpose in case of force/moment.
 ** The transpose is an approximation that can work if scaled appropriately.
 ** The latter is more stable that the pseudoinverse approach which struggles near singularities.
 **
 ** For a 6x7 non-square matrix:
 ** For position control:
 ** del_x = Jacobian * del_q
 ** dq = J# * del_x
 **
 ** For force control:
 ** dq=Jt * k* del_f 
 **
 ** Note: Matrices use column-major indexing.
 *************************************************************************************************************************************/
  bool controller::JacobianProduct(/*in*/ std::string type, /*out*/Eigen::VectorXd& update)
  {
    // Initializing a 6x7 jacobian and a 6D joint vector
    Eigen::MatrixXd jacobian;                               // Eigen defaults to storing the entry in column-major.
    Eigen::VectorXd ke = Eigen::VectorXd::Zero(6), dqbis;

    // 1. Get the 6x7 Jacobian for Baxter. 
    kine_model_->getJacobian(joints_[0], joints_names_, jacobian);

    // 2. Compute error x gain 
    if(type == "force")
      {
        for(unsigned int i=0; i<3; i++)
          ke(i) = gF_(i)*error_(i);
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          ke(i+3) = gM_(i)*error_(i+3);
      }
    else
      {
        ROS_ERROR("Could not recognize type of controller");
        return false;
      }

    // 3. Compute the delta angles=Jt*(k*wrench_error). 
    dqbis = jacobian.transpose() * ke;

    // Copy to output form
    update = dqbis;

    // Printout the delta joint angle update
    ROS_INFO_STREAM("\n-------------------------------\nDelta Joint Angle Update qs:\n-------------------------------\n" << update << "\n-------------------------------\n");
    return true;
  }

  // Primitive Controller. 
  // Output: contained in update. Is the different in joint angles that needs to be applied. 
  bool controller::computePrimitiveController(std::vector<Eigen::VectorXd>& update, std::string type, geometry_msgs::Vector3 desired, std::vector<double>& e)
  {
    double error_norm=0;

    // Init: Set vectors for actual and goal data
    Eigen::Vector3d curdata, goal(desired.x, desired.y, desired.z);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(joints_.size());

    // 1. Current data is obtained from getWrenchEndpoint
    curdata = getDesiredForce(type);

    // 2. Error between actual and desired wrench is computed, placed in error_
    error_norm = computeError(type, curdata, goal);
    e.push_back(error_norm);  // Keep track of norm. Helps us look at direction of error.
    ROS_INFO_STREAM("\n-------------------------------\n\t\tError Norm: " << error_norm << "\n-------------------------------\n\n\n");
	
    // 3. Compute the product of the error and the jacobian to produce the delta joint angle update. 
    if(!JacobianProduct(type, dq))
      {
        ROS_ERROR("Could not get Jacobian");
        return false;
      }

    // 4. Save the result in update. 
    update.push_back(dq);
    return true;
  }

  bool controller::NullSpaceProjection(std::vector<Eigen::VectorXd> updates, sensor_msgs::JointState& dq)
  {
    Eigen::MatrixXd outer;
    Eigen::VectorXd dq2, dq1;
    double inner = updates[0].dot(updates[0]);
    if(inner = 0)
      inner = 1.0;

    outer = (updates[0] * updates[0].transpose()) / inner;

    dq2 = (Eigen::MatrixXd::Identity(7, 7) - outer) * updates[1];

    for(unsigned int i =0; i<joints_.size(); i++)
      {
        dq1(i) = updates[0](i) + dq2(i);
        if( isnan(dq1(i)) )
          {
            ROS_ERROR("NAN value at joint: %s", joints_names_[i].c_str());
            return false;
          }
      }

    dq = fill(dq1);
    return true;
  }

  // Service Server
  // This server will receive its request via req. 
  // See all properties via rossrv show forceController/forceControl or look in the srv folder.
  // Required
  // ---------------------------------------------------------------------------
  //   int32 num_ctrls
  //   string[] type // "force" or "moment"
  //   geometry_msgs/Vector3[] desired // i.e. desired force or moment values.
  //    float64 x
  //    float64 y
  //    float64 z
  //   geometry_msgs/Vector3[] gains // if you want to change the default gains.
  //    float64 x
  //    float64 y
  //    float64 z
  // ---------------------------------------------------------------------------
  // And the result is of type res:
  // ---------------------------------------------------------------------------
  // float64[] error
  // sensor_msgs/JointState update_angles
  //   std_msgs/Header header
  //     uint32 seq
  //     time stamp
  //     string frame_id
  //   string[] name      // holds baxter joint names: elbow, should, wrist.
  //   float64[] position // holds a delta joint angle update 
  //   float64[] velocity // not used
  //   float64[] effort	  // not used
  // ---------------------------------------------------------------------------
  // After the new joint angles are obtained, these are passed to a position controller that moves the arm. We do this as long as the force/moment error remains. 
  bool controller::execute(forceControl::Request &req, forceControl::Response &res)
  {
    ROS_INFO("Received the force controller service call. Will compute joint angle upate and pass to position controller.");

    // Local variables
    bool ok = false;
    std::vector<double> error, js;
    std::vector<Eigen::VectorXd> dqs;

    // Initialize vectors
    js.resize(7);
    dqs.clear();
    error.clear();

    // Call the init function to...
    ini();
 
    // A1. Check for gain update
    if(req.gains.size() !=0)
      updateGains(req.gains, req.type);

    // Wait for the first joint angles readings
    while(!ok)
      {
        if(jo_ready_)
          ok = true;
      }

    // A2. Check service call errors: I.e. required number of controller and type size.
    if(req.desired.size() != req.num_ctrls || req.type.size() != req.num_ctrls)
      {
        ROS_ERROR("The type of controller and/or the desired values does not match the number of controllers!");
        return false;
      }

    // B. Call controllers. 
    // Do so as long as force/moment error is above a given threshold
    Eigen::Vector3d force_error;

    // Create desired vector. Currently passing the first element of the vector, ie desired[0]. 
    Eigen::Vector3d  desired_force = Eigen::Vector3d(req.desired[0].x, req.desired[0].y, req.desired[0].z); 
    for(int i=0; i<3; i++)
      force_error[i] = torque_[0][i] - desired_force[i];

    // Compute the squaredNorm: sqrt(|v|)
    double fe_norm=force_error.squaredNorm();
    
    // As long as our norm is greater than the force error threshold, continue the computation.
    
    // Set time rate
    ros::Rate loopRate(10);
    while(fe_norm > force_error_threshold_)
      {
        // B1. Only 1 controller: call primitive controller. Store the delta joint angle update in dqs. 
        for(unsigned int i=0; i<req.num_ctrls; i++)
          {
            ROS_INFO("Calling primitive controller %d", i);

            // Takes req.desired input  type. 
            // Outputs a delta joint angle in dqs and the error. 
            if(!computePrimitiveController(dqs, req.type[i], req.desired[i], error))
              {
                ROS_ERROR("Could not compute angle update for type: %s", req.type[i].c_str());
                return false;
              }
          }

        // B2. 2 Controllers: call compound controllers
        if(req.num_ctrls > 1)
          {
            // Projects dqs update to nullspace of primary controller and compute new dqs in res.update_angles. 
            if(!NullSpaceProjection(dqs, res.update_angles))
              {
                ROS_ERROR("Could not get null space projection");
                return false;
              }
          }
    
        // Add delta joint angles to current joint angles through fill.
        else
          res.update_angles = fill(dqs[0]); // TODO print to screen

        // Also save the errors
        res.error = error;

        // C. Move to desired joint angle position through a positon control loop
        position_controller(res.update_angles,to_);

        // Set frequency to 
        loopRate.sleep();

      } // While error_norm

    return true;	
  }

  //********************************************************************************************************************************************************************
  // position_controller()
  // Move to the desired joint angles. This function originally devised in the position_control package. 
  // Input: 
  // - update_angles.position with desired updated positions in the ordered set: {s0,s1,e0,e1,w0,w1,w2}
  // - original time for the program
  // 
  //********************************************************************************************************************************************************************
  void controller::position_controller(sensor_msgs::JointState qd, ros::Time t0)
  {
    // side_ = qd.header.frame_id;
    initialize();
	
    // Copy qd into goal_ and clear member qd_ 
    goal_.clear();	qd_.clear();                     // goal_ is of type vector<doubles> 
    for(unsigned int i=0; i<qd.position.size(); i++)
      goal_.push_back(qd.position[i]); //qd is JointStates. has a float[] positions
    qd_ = goal_;

    // Create a new variable qgoal in which we filter the joing angle between the current position joints_ and the goal position goal_. If alpha is 0, we send the goal directly, if alpha is 1, we stay in our current position. This filter has the effect of speeding up or slowing down the motion of the robot. 
    qgoal_.mode = qgoal_.POSITION_MODE;              // qgoal is of type baxter_core_msgs/JointCommand. Consists of int mode, float[] name, float[] command.
    qgoal_.names = joints_names_;                    // Make sure that the order of these names is {s0,s1,e0,e1,w0,w1,w2}
    qgoal_.command.resize( goal_.size() );

    for(unsigned int i=0; i< goal_.size(); i++)
      qgoal_.command[i] = ((1-alpha_)*goal_[i]) + (alpha_ * joints_[0][i]);

    // Get current time 
    ros::Time tnow = ros::Time::now();
    n_ = 0;

    // Publish desired filtered joint angles (arm moves)
    ROS_INFO("\n------------------------------\nBaxter's Joint Angle goal command is:\
              \n------------------------------\n<%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f> at time: %f\n------------------------------\n",
             qgoal_.command[0],qgoal_.command[1],qgoal_.command[2],
             qgoal_.command[3],qgoal_.command[4],qgoal_.command[5],
             qgoal_.command[6],(tnow-t0).toSec());
    // Publish to the topic /robot/limb/right/joint_command. Baxter will move upon receiving this command. 
    joint_cmd_pub_.publish(qgoal_);

    // Check to see if the arm has finished moving. 
    bool cont, fin = isMoveFinish(cont);

    // If not finished, keep looping until finished. 
    while(!fin && node_handle_.ok())
      {
        // Filter: update the goal command a bit closer to our goal. 
        for(unsigned int i=0; i<goal_.size(); i++)
          qgoal_.command[i] = ((1-alpha_)*goal_[i]) + (alpha_ * qgoal_.command[i]);	
     
        joint_cmd_pub_.publish(qgoal_);
        fin = isMoveFinish(cont);
        
        ros::Duration(0.05).sleep();
      }

    // Publish the error
    ROS_WARN("The current joint position error is: %f, %f, %f, %f, %f, %f, %f", qe_[0],qe_[1],qe_[2],qe_[3],qe_[4],qe_[5],qe_[6]);

    ROS_INFO("\n\n--------------------------------------------- Finished Moving the %s arm ---------------------------------------------\n\n", side_.c_str());
  }

  // isMoveFinish
  // Input: boolean result
  // Output: boolean result
  // Check to see if the arm has finished moving to desired joint angle position. This function is called 20 times in a sec
  bool controller::isMoveFinish(bool& result)
  {  
    double max=0.0;
    std::vector<double> error;
    error.clear();
    
    // Start counter
    n_ = n_ + 1;

    // This is true except in the first move and after 5 moves. 
    if( n_!=0  &&  (n_ % 5 != 0) )  
      {
        result = false;
        return false;
      }

    // Compute the error in position
    for(unsigned int i=0; i<joints_[0].size(); i++)
      {
        error.push_back( joints_[0][i] - qd_[i] );

        // Update position error threshold
        if ( fabs(error.back()) > max)
          max = error.back();
      }

    // Transfer info from qgoal_ to a local varaible. Test if error is going down for each joint.
    int ok=0, keep=0, k=0;
    std::vector<double> q;
    q.clear();	q.resize(joints_names_.size()); 
    bool copy;

    for(unsigned int i=0; i < joints_names_.size(); i++)
      {
        copy = true;
        if( k < qgoal_.names.size() )
          {
            // Copy qgoal to q[]
            if(qgoal_.names[k] == joints_names_[i]) 
              {
                q[i] = qgoal_.command[k]; 
                k = k + 1;
                copy = false;
              }
          }
        
        // Copy joints to q only if all goal info has been copies and clear qgoal.
        if(copy)
          q[i] = joints_[0][i];
      }	

    // Test if error is reducing
    qgoal_.names.clear();
    goal_.clear();
    qgoal_.command.clear();

    // If error > tolerance, set qgoal again. Need to test for each of the joints separately. 
    for(unsigned int i=0; i<joints_[0].size(); i++)
      {	       
        if( fabs(error[i]) > tolerance_)         // Tolerance comes from precision parameter in launch file or default value in constructor. Default is 0.001
          {
            qgoal_.names.push_back( joints_names_[i] );
            qgoal_.command.push_back(q[i]);
            goal_.push_back(qd_[i]);

            // Have we reached a termination condition? Still have errors?
            if( fabs( error[i] - qe_[i] ) != 0.0 )  
              keep = keep + 1; // up to 7 joints
          }
        else
          ok = ok + 1;
      }
    qe_ = error; // qe_ is a vector of doubles, error is a double

    //  Have a timer in case robot does not converge to desired position for all 7 joints.
    ros::Time tnow = ros::Time::now();
    int timeOut=30;
    if( (keep + ok == joints_.size()) && (ok != joints_names_.size()) && ( ((tnow - to_).toSec()) <= timeOut ))
      {
        // 200th step issue a warning.
        if(n_ % 200 == 0)
          ROS_WARN("time: %f, keep = %d, ok = %d", tnow.toSec(), keep, ok);

        result = true; // We have a result but we have not reached the goal. 
        return false;
      }

    // Successful case. All 7 joints are under tolerance.
    else if(ok == joints_names_.size())
      {
        ROS_INFO("Joint Position Error is under tolerance!");
        result = true;
        return true;
      }

    // If two or less joints have only reached their goal or timeOut is greater....
    else if( (keep <= 2) || ( ((tnow - to_).toSec()) > timeOut ) )
      {
        result = false;   // Could not get a result.
        ROS_ERROR("Time Out, t = %f, n = %d", (tnow - to_).toSec(), n_);
        return true;
      }

    if(n_ % 200 == 0)
      ROS_WARN("F: time: %f, keep = %d, ok = %d", tnow.toSec(), keep, ok);
	
    result = true;
    return false;
  }
}  //namespace controller


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Baxter_controller");

  // Create a Private Name (not global name). Will use <node_name>/my_private_namespace/my_private_topic in front of any topic/services/parameter created with this handle.
  ros::NodeHandle node("~"); 

  // Instantiate the controller
  force_controller::controller myControl(node);

  // Set up the dynamic reconfigure server
  dynamic_reconfigure::Server<force_error_constants::force_error_constantsConfig> srv;
  dynamic_reconfigure::Server<force_error_constants::force_error_constantsConfig>::CallbackType f;
  f=boost::bind(&force_controller::callback, _1, _2); // Used to pass two params to a callback.
  srv.setCallback(f);

  if(!myControl.start())
    {
      ROS_ERROR("Could not start controller, exiting");
      ros::shutdown();
      return 1;
    }
  ros::Duration(1.0).sleep();

  ros::MultiThreadedSpinner spinner(5); // One spinner per ROS communication object: here we use it for 
                                        // 1. Publish joint commands
                                        // 2. Subsribe to current joint Angles
                                        // 3. Advertice a service server
                                        // 4. Subscribe to endpoint wrench (optional)
                                        // 5. Dynamic Reconfigure
  spinner.spin();

  return 0;
}  
