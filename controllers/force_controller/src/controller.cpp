#include <force_controller/controller.h>
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
             config.k_fp0,
             config.k_fp1,
             config.k_fp2,
             config.k_mp0,
             config.k_mp1,
             config.k_mp2);
  
  // Save to the corresponding data members. 
  k_fp0=config.k_fp0;
  k_fp1=config.k_fp1;
  k_fp2=config.k_fp2;
  k_mp0=config.k_mp0;
  k_mp1=config.k_mp1;
  k_mp2=config.k_mp2;

  // change the flag
  force_error_constantsFlag = true;

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

    // Resize other kinematic model variables that need to be used in computeError
    joints_.clear();
    j_t_1_.clear();
    jv_t_1_.clear();

    j_t_1_.resize(joints_names_.size());
    jv_t_1_.resize(joints_names_.size());
    tm_t_1_.resize(joints_names_.size());
    tg_t_1_.resize(joints_names_.size());

    // Initialize qe_ to a high value on the first iteration: used in position_controller
    qe_.clear();
    for(unsigned int i=0; i<joints_names_.size(); i++)
      qe_.push_back(100.0);
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

  //*************************************************************************
  // fill(...)
  // Takes a delta joint angle update and add it to the current angles. TODO: make sure you know the order of the joints strings.
  // Current angles are set in getBaxterJointState where filtered values are set to joints_[0]
  //*************************************************************************
  sensor_msgs::JointState controller::fill(Eigen::VectorXd dq)
  {
    // Create a JointState structure. Composed of string[] name and float64[] pos/vel/effort.
    sensor_msgs::JointState update;
    update.name = joints_names_;
    update.position.clear();

    // Add delta joint angle updates to current angles held in joints_[0]. 
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
  // extractWrench_Force_Moment()
  // Extracts either force or moment from wrench based on the input string. It assumes that a subscriber for the wrench data is being called and that the data is available.
  // Input: string for force or moment.
  // Returns 3D vector.
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
  Eigen::Vector3d controller::extractWrench_Force_Moment(string type)
  {
    Eigen::Vector3d wrench = Eigen::Vector3d::Zero(); 

    // Split values into force or moment
    if(type == "force")
      {
        for(unsigned int i=0; i<3; i++)
          {
            if(wrenchFilteringFlag)
              wrench(i) = cur_data_f_(i);
            else
              wrench(i) = cur_data_(i);
          }
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          {
            if(wrenchFilteringFlag)
              wrench(i) = cur_data_f_(i+3);
            else
              wrench(i) = cur_data_(i+3);
          }
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
  // Also available is the choice to filter the data using a 2nd order low-pass filter.
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  void controller::getWrenchEndpoint(const baxter_core_msgs::EndpointStateConstPtr& state)
  {  
    // Local variables. Used in 2. 
    // Eigen::Vector3d result = Eigen::Vector3d::Zero();
    Eigen::VectorXd offset = getTorqueOffset(), realTorque; // wrench; // Create 3 vectors: offset, realTorque, and wrench (force error). 
    Eigen::MatrixXd jacobian, JJt;

    // Assign the correct size for the vectors. 
    realTorque = Eigen::VectorXd::Zero(7);
    // wrench     = Eigen::VectorXd::Zero(6);
    
    // Select a method. False if you want a Jacobian computation for torques->wrench. True to get wrench from topic.
    if(wrench_sub_flag) {
      
      // Get the force and torque from the callback argument state
      cur_data_ << state->wrench.force.x, state->wrench.force.y, state->wrench.force.z,
                   state->wrench.torque.x,state->wrench.torque.y,state->wrench.torque.z;

      // Offset Removal. TODO: Pending to do any such analysis, but it would go here.
      // 
    }

    // Convert from joint torques to wrench.
    else {
      // Real Torque = sensed torque - gravitational torque - modeled offset torque. 
      for(unsigned int i=0; i<7; i++)
        realTorque(i) = torque_[0][i] - tg_[0][i]; // - offset(i);

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
      ROS_INFO_STREAM("\n-------------------------------\nThe current wrench value is:\n-------------------------------\n" << cur_data_ << "\n-------------------------------\n");
      // Use Eigen's least square approach: 
      // wrench = Jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(realTorque);
      for(unsigned int i=0; i<6; i++)
        {
          if(isnan(cur_data_(i)))
            {
              ROS_ERROR_STREAM("Failed to compute Jacobian Pseudo Inverse " << cur_data_(i) << ", tor " << realTorque << ", jac" << jacobian << ", j " << joints_[0][0]);
              // return cur_data_;
            }
        }
    } // End if-else for two methods.   

    /***************************************************************************** Filtering ************************************************************************************/
    // Create a vector or 6D Eigen vectors
    Eigen::VectorXd temp = Eigen::VectorXd::Zero(6);    

    if(wrenchFilteringFlag)
      {
        // If this is the first loop we need to insert two rows with equal values as the current iteration for both wrenchVec and wrenchVecF
        if(initialFiltering)
          {
            for(int i=0; i<2; i++)
              {
                // Input wrench at the end of the vector
                wrenchVec.push_back(cur_data_);               

                // Filtered wrench at the end of the vector
                wrenchVecF.push_back(cur_data_);
              }
            // Change Flag
            initialFiltering=false;
          }

        // Copy new data to wrenchVec at the end of the vector
        wrenchVec.push_back(cur_data_);
        
        // Assuming a vector of size(3:0,1,2) Element [2] is the current one (sitting at the back), element t-1 is [1], sitting in the middle, and element t-2 is[0]. 
        // The indeces are opposite to what you think it should be. 
        for(int i=0; i<6; i++)
          temp(i) = 0.018299*wrenchVec[2](i) + 0.036598*wrenchVec[1](i) + 0.018299*wrenchVec[0](i) + 1.58255*wrenchVecF[1](i) - 0.65574*wrenchVecF[0](i);

        // Add new filtered result
        wrenchVecF.push_back(temp);

        // Pop last value from these two structures to keep the size of the vector to 3 throughout
        wrenchVec.pop_front();
        wrenchVecF.pop_front();
      }

    // Set filterd value to private member
    for(int i=0; i<6; i++)
      cur_data_f_(i) = wrenchVecF[0](i);

    baxter_core_msgs::EndpointState fw;
    // Time Stamp
    fw.header.stamp = ros::Time::now();

    // Wrench
    fw.wrench.force.x  = cur_data_f_(0); 
    fw.wrench.force.y  = cur_data_f_(1); 
    fw.wrench.force.z  = cur_data_f_(2); 
    fw.wrench.torque.x = cur_data_f_(3); 
    fw.wrench.torque.y = cur_data_f_(4); 
    fw.wrench.torque.z = cur_data_f_(5); 

    // Republish. Leading to system crash..
    if(filtered_wrench_pub_flag)
      {
        filtered_wrench_pub_.publish(fw);
        // ROS_INFO_STREAM("Publishing the filtered wrench: " << fw.wrench << std::endl);
      }
  }

  //***************************************************************************************************************************************************
  // getBaxterJointState(...)
  // 
  // This callback function works in hand with the subscription call joints_sub_ using the topic /robot/limb/"side"/gravity_compensation_torques
  // From this topic we obtain current joint angle positions, velocities, actual torques, and gravity_model_effort torques (compensation + hysteresis + crosstalk).
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
  //***************************************************************************************************************************************************
  void controller::getBaxterJointState(const baxter_core_msgs::SEAJointStateConstPtr& state)
  {
    // Local variables
    ros::Time t = ros::Time::now();
    unsigned int i, k=0;

    // Create Joint, Torque, and Gravitation values and filtered values. 
    std::vector<double> jt, jtf;    // Joints and Filtered Joints
    std::vector<double> jv, jvf;    // Joint Velocities and Filtered Joint Velocities
    std::vector<double> tt, tg;     // Torques and Filtered Torques
    std::vector<double> ttf, tgf;   // Gravitational Torques and Filtered equivalents.

    // Joint Angles
    jt.clear();		jtf.clear();
    jt.resize(joints_names_.size());
    jtf.resize(joints_names_.size());

    // Joint Velocities
    jv.clear();   jvf.clear();
    jv.resize(joints_names_.size());
    jvf.resize(joints_names_.size());

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
          { // Joints names order is: s0s1e0e1w0w1w2. The name list from the topic may not always be in this order. This is necessary to make sure we put the right value in the right place.            
            if(state->name[i] == joints_names_[k])
              {
                // Get current joint angles
                jt[k] = state->actual_position[i];

                // Get current joint Velocities
                jv[k] = state->actual_velocity[i];

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
        // Save current values into filtered parameters to start
        for(unsigned int i=0; i<joints_names_.size(); i++)
          {
            ttf[i] = tt[i];
            tgf[i] = tg[i];

            // Create previous (unfiltered) values
            tm_t_1_[i] = tt[i];
            tg_t_1_[i] = tg[i];
            j_t_1_[i]  = jt[i];
            jv_t_1_[i] = jv[i];
          }

        // Push 2x!! (to create a 2nd order system) to private members before computing (lowpass) IIR Filtering
        torque_.push_back(tm_t_1_);
        torque_.push_back(tm_t_1_);
        
        tg_.push_back(tg_t_1_);
        tg_.push_back(tg_t_1_);
        
        joints_.push_back(jt);
        joints_.push_back(jt);

        velocity_.push_back(jv);
        velocity_.push_back(jv);
        
        jo_ready_ = true;
      }

    // IIR Low Pass Filter: create the filtered values.
    for(unsigned int i=0; i<7; i++)
      {
        jtf[i] = 0.0784*j_t_1_[i]  + 1.5622*joints_[0][i]   - 0.6413*joints_[1][i];
        jvf[i] = 0.0784*jv_t_1_[i] + 1.5622*velocity_[0][i] - 0.6413*velocity_[1][i];
        ttf[i] = 0.0784*tm_t_1_[i] + 1.5622*torque_[0][i]   - 0.6413*torque_[1][i];
        tgf[i] = 0.0784*tg_t_1_[i] + 1.5622*tg_[0][i]       - 0.6413*tg_[1][i];
        
        j_t_1_[i]  = jt[i];
        jv_t_1_[i] = jv[i];
        tm_t_1_[i] = tt[i];
        tg_t_1_[i] = tg[i];

        // Save filtered value to private members
        joints_[1][i]   = joints_[0][i];         // Make index 1, older element
        joints_[0][i]   = jtf[i];                // Save filtered val to head of vec

        velocity_[1][i] = velocity_[0][i];
        velocity_[0][i] = jvf[i];        

        torque_[1][i] = torque_[0][i];
        torque_[0][i] = ttf[i];

        tg_[1][i]     = tg_[0][i];
        tg_[0][i]     = tgf[i];
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

  //****************************************************************************************************
  // updateGains(...)
  // Used to update gains at run-time through the parameter server or through rqt_reconfigure.
  //****************************************************************************************************  
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
          gFp_ = Eigen::Vector3d(gain[i].x, gain[i].y, gain[i].z);
        else if(type[i] == "moment")
          gMp_ = Eigen::Vector3d(gain[i].x, gain[i].y, gain[i].z);
        else
          ROS_WARN("Could not recognize type of controller, using default gain value");
      }
  }

  void controller::updateGains() {

    // Update with rqt_reconfigure updated parameters
    gFp_ << k_fp0, k_fp1, k_fp2;
    gMp_ << k_mp0, k_mp1, k_mp2;

    // change the flag
    force_error_constantsFlag = false;

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
          ke(i) = gFp_(i)*error_(i) + gFv_(i)*derror_(i);
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          ke(i+3) = gMp_(i)*error_(i+3);
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


  //*******************************************************************************************************
  // Same as above, but I want to multiply by the gains after the jacobian product to see if this makes any difference. So far in my testing, arm just moves to become straight. Does not seem to work. 
  //*******************************************************************************************************
  bool controller::JacobianErrorProduct(/*in*/ std::string type, /*out*/Eigen::VectorXd& update)
  {
    // Initializing a 6x7 jacobian and a 6D joint vector
    Eigen::MatrixXd jacobian;                               // Eigen defaults to storing the entry in column-major.
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd dq_scaled = Eigen::VectorXd::Zero(7);

    // 1. Get the 6x7 Jacobian for Baxter. 
    kine_model_->getJacobian(joints_[0], joints_names_, jacobian);

    // 2. Compute the delta angles=Jt*(k*wrench_error). 
    dq = jacobian.transpose() * error_;

    // 3. Compute error x product 
    if(type == "force")
      {
        // for(unsigned int i=0; i<3; i++)
        dq_scaled = gFp_(0)*dq;// gain * outoput of product (7x1)
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          dq_scaled(i+3) = gMp_(0)*dq(i+3);
      }
    else
      {
        ROS_ERROR("Could not recognize type of controller");
        return false;
      }
    
    // Copy to output form
    update = dq_scaled;

    // Printout the delta joint angle update
    ROS_INFO_STREAM("\n-------------------------------\nDelta Joint Angle Update qs:\n-------------------------------\n" << update << "\n-------------------------------\n");
    return true;
  
}
  //************************************************************************************************************************************************************
  // computePrimitiveController(...)
  // Output: places commanded joint angles in update. 
  //************************************************************************************************************************************************************
  bool controller::computePrimitiveController(std::vector<Eigen::VectorXd>& update, std::string type, Eigen::Vector3d setPoint, std::vector<double>& e)
  {
    // Init: Set vectors for actual and goal data
    Eigen::Vector3d curdata;
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(joints_.size());

    // 1. Current data is obtained from getWrenchEndpoint
    curdata = extractWrench_Force_Moment(type);

    // 2. Error between actual and desired wrench is computed, placed in error_
    error_norm_ = computeError(type, curdata, setPoint);
    e.push_back(error_norm_);  // Keep track of norm. Helps to identify direction of controller
    ROS_INFO_STREAM("Error Norm: " << error_norm_);
	
    // 3. Compute the product of the error and the jacobian to produce the delta joint angle update placed in dq.
    if(!JacobianProduct(type, dq)) 
      {
        ROS_ERROR("Could not get Jacobian");
        return false;
      }

    // 4. Save the result as the first vector element of update.
    // update.push_back(dq);
    update[0]=dq;
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
  bool controller::force_controller(forceControl::Request &req, forceControl::Response &res)
  {
    ROS_INFO("Received the force controller service call. Will compute joint angle upate and pass to position controller.");

    // Local variables
    bool fin= true;
    bool ok = false;
    std::vector<double> error, js;
    std::vector<Eigen::VectorXd> dqs;

    // Set initial variables like time t0_=0.0
    // side_ = qd.header.frame_id;
    initialize();

    // Initialize vectors
    js.resize(7);                    // Will keep current 7 joint angles here
    dqs.resize(1);                   // Here we will only keep 1 copy of joint angle updates
    dqs[0]=Eigen::VectorXd::Zero(7); // TODO might want to change dqs to simply be an eigen vector. change the prototype of primitiveController and NullSpaceProjection. Is there a need for the vector and the history?
    error.clear();

    // Call the init function to...
    ini();
 
    // A1. Check for gain updates:
    // From command line or roslaunch
    if(req.gains.size() !=0)
      updateGains(req.gains, req.type);

    // Update from Dynamic Reconfigure GUI
    else if(force_error_constantsFlag)
      updateGains();

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

    // Set time rate
    ros::Rate loopRate(while_loop_rate_);

    // B. Call controllers. 
    // Do so as long as force/moment error is above a given threshold
    // Set the desired force/moment.
    setPoint_ << req.desired[0].x, req.desired[0].y, req.desired[0].z;
 
    // As long as our norm is greater than the force error threshold, continue the computation.
    //    do {
      // B1. Only 1 controller: call primitive controller. Store the delta joint angle update in dqs. 
      for(unsigned int i=0; i<req.num_ctrls; i++)
        {
          ROS_INFO("Calling primitive controller %d", i);

          // Takes req.desired input  type. 
          // Outputs a delta joint angle in dqs and the error. 
          if(!computePrimitiveController(dqs, req.type[i], setPoint_, error))
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
    
      // Add delta joint angles to current joint angles through fill, store in res.update_angles.position
      else
        res.update_angles = fill(dqs[0]); // this should not be 0!
        
      // Also save the errors
      res.error = error;

      // C. Move to desired joint angle position through a positon or torque control loop
      if(jntPos_Torque_InnerCtrl_Flag_)           
        fin=position_controller(res.update_angles,to_); // Position Controller
        
      else 
        torque_controller(dqs[0],to_);              // Torque Controller        

    //   // If position controller did not finish properly, exit, else continue the while loop.
    //   if(!fin)
    //     break;

    //   // Set frequency to 
    //   loopRate.sleep();
    //   }  while(fin && error_norm_ > force_error_threshold_); // do while
    // ROS_INFO_STREAM("isMoveFinish returns: " << fin );

    return true;	
  }

  //********************************************************************************************
  // torque_controller(...)
  // Compute error in wrench point, and then use the Jt and a gain to compute torque updates.
  // Sent to /joint_command
  //********************************************************************************************
  void controller::torque_controller(Eigen::VectorXd delT, ros::Time t0)
    {
      // Copy qd into goal_ and clear member qd_ 
      goal_.clear();	qd_.clear();                     // goal_ is of type vector<doubles> 
      for(unsigned int i=0; i<7; i++)
        goal_.push_back(delT[i]);
      qd_ = goal_;

      qgoal_.mode = qgoal_.TORQUE_MODE;               // qgoal is of type baxter_core_msgs/JointCommand. Consists of int mode, float[] name, float[] command.
      qgoal_.names = joints_names_;                   
      qgoal_.command.resize( goal_.size() );

      // Compute commanded torque: actual_torque - gravity_model_effort + delta torque.
      // Notice that the first two are vectors, so we take the top vector elements.
      for(unsigned int i=0; i< goal_.size(); i++)
        qgoal_.command[i] = torque_[0][i]-tg_[0][i]+delT[i]; // Actual torque - gravity compensation (will be added) + delta 
        // qgoal_.command[i] = torque_[0][i]+delT[i]; // Trying with gravity compensation suppresed resulted in a fast and dangerous motion dropping the arm.

      // Get current time 
      ros::Time tnow = ros::Time::now();
  
      // Publish desired filtered joint angles (arm moves)
      ROS_INFO("Commanded torque is:\
              \n------------------------------\n<%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f> at time: %f\n------------------------------\n",
               qgoal_.command[0],qgoal_.command[1],qgoal_.command[2],
               qgoal_.command[3],qgoal_.command[4],qgoal_.command[5],
               qgoal_.command[6],(tnow-t0).toSec());

      // Publish to the topic /robot/limb/right/joint_command. Baxter will move upon receiving this command. 
      joint_cmd_pub_.publish(qgoal_);
      ROS_INFO("\n--------------------------------------------- Finished Moving the %s arm ---------------------------------------------\n\n", side_.c_str());      
    }

  //********************************************************************************************************************************************************************
  // position_controller()
  // Input: JointState qd is the refernce angles where we want to go. t0 is the initial time of the entire program in the ordered set: {s0,s1,e0,e1,w0,w1,w2}
  // Try to move to the desired joint angles. This function originally devised in the position_control package. 
  // Input: 
  // - update_angles.position with desired updated positions 
  //********************************************************************************************************************************************************************
  bool controller::position_controller(sensor_msgs::JointState qd, ros::Time t0)
  {
    // Copy qd into goal_ and clear member qd_ 
    goal_.clear();	qd_.clear();                     // goal_ is of type vector<doubles> 
    for(unsigned int i=0; i<qd.position.size(); i++)
      goal_.push_back(qd.position[i]);               // push back the reference angles. qd is JointStates and has a position[]
    qd_ = goal_;

    // Create a new variable qgoal in which we filter the joing angle between the current position joints_ and the goal position goal_. If alpha is 0, we send the goal directly, if alpha is 1, we stay in our current position. This filter has the effect of speeding up or slowing down the motion of the robot. 
    qgoal_.mode = qgoal_.RAW_POSITION_MODE;              // qgoal is of type baxter_core_msgs/JointCommand. Consists of int mode, float[] name, float[] command.
    qgoal_.names = joints_names_;                    // Make sure that the order of these names is {s0,s1,e0,e1,w0,w1,w2}
    qgoal_.command.resize( goal_.size() );

    for(unsigned int i=0; i< goal_.size(); i++)
      qgoal_.command[i] = ((1-alpha_)*goal_[i]) + (alpha_ * joints_[0][i]);

    // Get current time 
    ros::Time tnow = ros::Time::now();
    n_ = 0;

    // Publish desired filtered joint angles (arm moves). The time is the diff from now to the beginning of the demo.
    ROS_INFO("Commanded Joint Angle:\
              \n------------------------------\n<%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f> at time: %f\n------------------------------",
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
     
        // Print commanded joint angles again and their current time 
        ros::Time tnow = ros::Time::now();
        ROS_INFO("Commanded Joint Angle:\
              \n------------------------------\n<%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f> at time: %f\n------------------------------",
             qgoal_.command[0],qgoal_.command[1],qgoal_.command[2],
             qgoal_.command[3],qgoal_.command[4],qgoal_.command[5],
             qgoal_.command[6],(tnow-t0).toSec());
        
        joint_cmd_pub_.publish(qgoal_);
        fin = isMoveFinish(cont);
        
        ros::Duration(0.005).sleep();

        return fin;
      }

    // Print error to screen
    ROS_WARN("The current joint position error is: %f, %f, %f, %f, %f, %f, %f", qe_[0],qe_[1],qe_[2],qe_[3],qe_[4],qe_[5],qe_[6]);

    ROS_INFO("\n---------------------------------------------\nFinished Moving the %s arm\n---------------------------------------------\n", side_.c_str());
  }

  //*************************************************************************************************************************
  // isMoveFinish(...)
  // Input: boolean output to determine if we should continue with this function. 
  // Output: boolean output to determine if we have reached our goal.
  // Check to see if the arm has finished moving to desired joint angle position. This function is called at a given rate according to ros duration. 
  // Uses qgoal_ which is originally set in the force_controller. It's of type baxter_core_msgs/JointCommand and contains a mode, command[], and names[].
  //*************************************************************************************************************************
  bool controller::isMoveFinish(bool& result)
  {  
    double max=0.0;
    std::vector<double> error; error.clear();
    
    // Start counter: used to measure how many times we attempt to move before reaching the goal.
    n_ = n_ + 1;

    // If 5 tries are reached and we have not reached the goal, exit and set result to false. If this number is reached we say the result failed and also return false for isMoveFinish.
    if( n_!=0  &&  (n_ % 5 != 0) )  
      {
        result = false;
        return false;
      }

    // Record maximum error across all joints.
    for(unsigned int i=0; i<joints_[0].size(); i++)
      {
        // Joint error between current position and our qgoal_ var (filtered goal var)
        error.push_back( joints_[0][i] - qd_[i] );

        // Update position error threshold
        if ( fabs(error.back()) > max)
          max = error.back();
      }

    // Copy the reference qgoal_ to a local varaible q[i]. 
    // Test if error is going down for each joint.
    int ok=0, keep=0, k=0; // keep: means joints have not reached goal, ie keep trying.
    std::vector<double> q;
    q.clear();	q.resize(joints_names_.size()); 
    bool copy;

    // Mechanism to either copy current joint angles or goal.
    for(unsigned int i=0; i < joints_names_.size(); i++)
      {
        copy = true;
        // Copy goal
        if( k < qgoal_.names.size() )
          {
            // Only do the following when both names match. Ensures right order.
            if(qgoal_.names[k] == joints_names_[i]) 
              {
                q[i] = qgoal_.command[k]; 
                k = k + 1;
                copy = false;
              }
          }
        
        // Copy current joint angles only if goal info has been copied
        if(copy)
          q[i] = joints_[0][i];
      }	

    // Completely clear our reference goal joint names and command angles.
    qgoal_.names.clear();
    qgoal_.command.clear();

    // Have a new local variable goal be readied.
    goal_.clear();
    
    // If error > tolerance (test for each joint), set qgoal again. 
    for(unsigned int i=0; i<joints_[0].size(); i++)
      {	       
        // If Joint Position Error is greater than tolearnce, do again. 
        if( fabs(error[i]) > tolerance_)         // Tolerance comes from precision parameter (launch file/constructor value).
          {
            qgoal_.names.push_back( joints_names_[i] );
            qgoal_.command.push_back(q[i]);      // Adjusted in previous for/if loop
            goal_.push_back(qd_[i]);             // Orig reference angles

            // Have we reached a termination condition? Still have errors?
            if( fabs( error[i] - qe_[i] ) != 0.0 )  
              keep = keep + 1; // If keep reaches > 0, some joints did not reach goal
          }
        else
          ok = ok + 1;
      }
    qe_ = error; // qe_ is a vector of doubles, error is a double

    //  Have a timer in case robot does not converge to desired position for all 7 joints.
    // The timeout is measured from the time the force_controller started until now. 
    ros::Time tnow = ros::Time::now();

    int maxCycles=timeOut_*while_loop_rate_;
    if( (keep + ok == joints_.size()) && (ok != joints_names_.size()) && ( ((tnow - to_).toSec()) <= timeOut_ ))
      {
        // Issue a warning when timeout has been reached. 
        if(n_ % maxCycles == 0)
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
    else if( (keep <= 2) || ( ((tnow - to_).toSec()) > timeOut_ ) )
      {
        result = false;   // Could not get a result.
        ROS_ERROR("Time Out, t = %f, n = %d", (tnow - to_).toSec(), n_);

        // Terminate the isMoveFinish logic.
        return true;
      }

    if(n_ % 200 == 0)
      ROS_WARN("F: time: %f, keep = %d, ok = %d", (tnow-to_).toSec(), keep, ok);
	
    result = true;
    return false;
  }
}  //namespace controller


int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_basis");

  // Create a Private Name (not global name). Will use <node_name>/my_private_namespace/my_private_topic in front of any topic/services/parameter created with this handle.
  ros::NodeHandle node("~"); 

  // Instantiate the controller
  force_controller::controller myControl(node);

  if(myControl.dynamic_reconfigure_flag)
    {
      // (i) Set up the dynamic reconfigure server
      dynamic_reconfigure::Server<force_error_constants::force_error_constantsConfig> srv;

      // (ii) Create a callback object of type force_error_constantsConfig
      dynamic_reconfigure::Server<force_error_constants::force_error_constantsConfig>::CallbackType f;

      // (iii) Bind that object to the actual callback function
      f=boost::bind(&force_controller::callback, _1, _2); // Used to pass two params to a callback.

      // Note: how to bind a private member function: If we cant callback to be a member method of the class controller, then we would need to use something like... 
      // boost::function<void (force_error_constants::force_error_constantsConfig &,int)> f2( boost::bind( &myclass::fun2, this, _1, _2 ) );
      // Still not clear on the syntax. Since it's not a member method, we make it a global and also need to use global parameters

      // (iv) Set the callback to the service server. 
      srv.setCallback(f);

      // Update the rosCommunicationCtr
      myControl.rosCommunicationCtrUp();
    }

  if(!myControl.start())
    {
      ROS_ERROR("Could not start controller, exiting");
      ros::shutdown();
      return 1;
    }
  ros::Duration(1.0).sleep();

  /*** Different Communication Modes ***/
  
  // 1. AsyncSpinner
  //ros::AsyncSpinner spinner(myControl.get_rosCommunicationCtr());
  //spinner.start();
  //ros::waitForShutdown(); 

  // 2. MultiThreadedSpinner
  ros::MultiThreadedSpinner spinner(myControl.get_rosCommunicationCtr()); // One spinner per ROS communication object: here we use it for 
                                                          // 1. Publish joint commands
                                                          // 2. Subsribe to current joint Angles
                                                          // 3. Advertice a service server
                                                          // 4. Subscribe to endpoint wrench (optional set in getWrenchEndpoint)
                                                          // 5. Dynamic Reconfigure (off)
                                                          // 6. published filtered wrench
  spinner.spin();
  ros::waitForShutdown(); 

  // // 3. Blocking Spin
  // ros::spin();
  // ros::waitForShutdown();

  return 0;
}  
