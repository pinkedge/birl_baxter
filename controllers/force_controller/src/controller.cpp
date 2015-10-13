#include <force_controller/controller.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

namespace force_controller
{
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
    joints_.clear();
    j_t_1_.clear();
    j_t_1_.resize(joints_names_.size());
    tm_t_1_.resize(joints_names_.size());
    tg_t_1_.resize(joints_names_.size());
  }

  std::vector<double> controller::toVector(const geometry_msgs::Vector3& d)
  {
    std::vector<double> result;
    result.clear();
    result.push_back(d.x);
    result.push_back(d.y);
    result.push_back(d.z);
    return result;
  }

  // Takes a delta joint angle update and adds it to the current angles.
  sensor_msgs::JointState controller::fill(Eigen::VectorXd dq)
  {
    sensor_msgs::JointState update;
    update.name = joints_names_;
    update.position.clear();

    // Add delta joint angle updates to current angles.
    for(unsigned int i=0; i<joints_names_.size(); i++)
      update.position.push_back(dq(i)+joints_[0][i]);

    // Clear other quantities. 
    update.velocity.clear();
    update.effort.clear();

    // Add a time stamp. 
    update.header.stamp = ros::Time::now();

    return update;
  }

  Eigen::VectorXd controller::getTorqueOffset()
  {
    Eigen::VectorXd torqueOffset = Eigen::VectorXd::Zero(7);
    if(side_ == "left")
      {
        for(unsigned int i=0; i<7; i++)
          torqueOffset(i) = COEFF[LEFT][i][0]*joints_[0][i]*joints_[0][i] + COEFF[LEFT][i][1]*joints_[0][i] + COEFF[LEFT][i][2];
      }
    else if(side_ == "right")
      {
        for(unsigned int i=0; i<7; i++)
          torqueOffset(i) = COEFF[RIGHT][i][0]*joints_[0][i]*joints_[0][i] + COEFF[RIGHT][i][1]*joints_[0][i] + COEFF[RIGHT][i][2];
      }
    else
      ROS_ERROR("Wrong arm name");

    return torqueOffset;
  }

  // Compute the Wrench at the Endpoint. 
  // Uses an offset computed a priori that tries to model the noise in Baxter's arms. 
  Eigen::Vector3d controller::getWrenchEndpoint(std::string type)
  {
    
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    Eigen::VectorXd offset = getTorqueOffset(), realTorque, fe;
    Eigen::MatrixXd jacobian, JJt;

    realTorque = Eigen::VectorXd::Zero(7);
    fe = Eigen::VectorXd::Zero(6);
    
    // Real Torque = sense torque - gravitational torque - modeled offset torque. 
    for(unsigned int i=0; i<7; i++)
      realTorque(i) = torque_[0][i] - tg_[0][i] - offset(i);

    // Print offset and real torques
    ROS_WARN_STREAM("Offset: " << offset);
    ROS_WARN_STREAM("Real: " << realTorque);

    // Get Jacobian
    kine_model_->getJacobian(joints_[0], joints_names_, jacobian);

    // Null Space Operator
    JJt = jacobian * jacobian.transpose();

    // Null Space Projection of Real Torque
    fe = (JJt.inverse() * jacobian) * realTorque;
    for(unsigned int i=0; i<6; i++)
      {
        if(isnan(fe(i)))
          {
            ROS_ERROR_STREAM("Failed to compute Jacobian Pseudo Inverse " << fe << ", tor " << realTorque << ", jac" << jacobian << ", j " << joints_[0][0]);
            return result;
          }
      }

    // Split values into force or moment
    if(type == "force")
      {
        for(unsigned int i=0; i<3; i++)
          result(i) = fe(i);
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          result(i) = fe(i+3);
      }
    else
      ROS_ERROR("Type is incorrect");

    return result;
  }

  // updateJoints
  // 
  // std_msgs/Header header
  // uint32 seq
  // time stamp
  // string frame_id
  // string[] name
  // ---
  // float64[] commanded_position      
  // float64[] commanded_velocity
  // float64[] commanded_acceleration
  // float64[] commanded_effort
  // ---
  // float64[] actual_position         // Current joint angle position in radians
  // float64[] actual_velocity
  // float64[] actual_effort           // This includes the inertial feed forward torques when applicable.
  // float64[] gravity_model_effort    // This is the torque required to hold the arm against gravity returned by KDL if the arm was stationary.  
  //                                   // This does not include inertial feed forward torques (even when we have them) or any of the corrections (i.e. spring
  // hysteresis, crosstalk, etc) we make to the KDL model.
  // float64[] gravity_only
  // float64[] hysteresis_model_effort
  // float64[] crosstalk_model_effort
  // float64 hystState
  // -------------------------------------------------------------------------------
  void controller::updateJoints(const baxter_core_msgs::SEAJointStateConstPtr& state)
  {
    // Local variables
    ros::Time t = ros::Time::now();
    unsigned int i, k=0;

    // Joint, Torque, and Gravitation Torque  values
    std::vector<double> jt, jtf, tt, tg, ttf, tgf;

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

    // Update the current joint values, actual effort, and gravity_model_effort values
    ROS_INFO_ONCE("Initializing joints");
    if(exe_ && save_.is_open())
      save_ << (t - to_).toSec() << "	";

    while(k < joints_names_.size())
      {
        for(i=0; i<state->name.size(); i++)
          {
            if(state->name[i] == joints_names_[k])
              {
                jt[k] = state->actual_position[i];
                tt[k] = state->actual_effort[i];
                tg[k] = state->gravity_model_effort[i];
                if(exe_ && save_.is_open())		  
                  save_ << state->actual_position[i] << "	";

                k = k + 1;
                if(k == joints_names_.size())
                  break;
              }
          }
      }

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

        // Savlue previous values
        torque_.push_back(tm_t_1_);
        torque_.push_back(tm_t_1_);
        
        tg_.push_back(tg_t_1_);
        tg_.push_back(tg_t_1_);
        
        joints_.push_back(jt);
        joints_.push_back(jt);
        
        jo_ready_ = true;
      }

    // Compute the offsets
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

    if(exe_ && save_.is_open())
      {
        for(unsigned int j=0; j<joints_.size(); j++)
          save_ << tt[j] << "	";
        for(unsigned int j=0; j<joints_.size(); j++)
          save_ << tg[j] << "	";
        save_ << std::endl;
      }

    ROS_INFO_STREAM_ONCE("Joints updated, tor: " << ttf[0] << ", "<< ttf[1] << ", "<< ttf[2]);
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

  // 
  bool controller::JacobianProduct(std::string type, Eigen::VectorXd& update)
  {
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd ke = Eigen::VectorXd::Zero(6), dqbis;

    kine_model_->getJacobian(joints_[0], joints_names_, jacobian);
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

    dqbis = jacobian.transpose() * ke;
    update = dqbis;
    ROS_INFO_STREAM("update qs: " << update);
    return true;
  }

  // Primitive Controller. 
  // Output: contained in update. Is the different in joing angles that needs to be applied. 
  bool controller::ComputePrimitiveController(std::vector<Eigen::VectorXd>& update, std::string type, geometry_msgs::Vector3 desired, std::vector<double>& e)
  {
    Eigen::Vector3d curdata, goal(desired.x, desired.y, desired.z);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(joints_.size());

    double error;
    curdata = getWrenchEndpoint(type);
    error = computeError(type, curdata, goal);
    e.push_back(error);
	
    // Compute the product of the error and the jacobian to produce the delta joint angle update. 
    if(!JacobianProduct(type, dq))
      {
        ROS_ERROR("Could not get Jacobian");
        return false;
      }

    // Save the result in update. 
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
  // req is of type forceController/forceControl: 
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
  bool controller::execute(forceControl::Request &req, forceControl::Response &res)
  {
    ROS_INFO("Received service call");
    bool ok = false;
    std::vector<double> error, js;
    std::vector<Eigen::VectorXd> dqs;

    // Initialize vectors
    dqs.clear();
    error.clear();
    ini();
    js.resize(7);

    // Check for gain update
    if(req.gains.size() !=0)
      updateGains(req.gains, req.type);

    // Wait for the first joint angles readings
    while(!ok)
      {
        if(jo_ready_)
          ok = true;
      }

    // Check for errors in required number of controller and type size.
    if(req.desired.size() != req.num_ctrls || req.type.size() != req.num_ctrls)
      {
        ROS_ERROR("The type of controller and/or the desired values does not match the number of controllers!");
        return false;
      }

    // A. Only 1 controller: call primitive controller. Store the delta joint angle update in dqs. 
    for(unsigned int i=0; i<req.num_ctrls; i++)
      {
        ROS_INFO("Calling primitive controller %d", i);
        if(!ComputePrimitiveController(dqs, req.type[i], req.desired[i], error))
          {
            ROS_ERROR("Could not compute angle update for type: %s", req.type[i].c_str());
            return false;
          }
      }

    // B. 2 Controllers: call compound controllers
    if(req.num_ctrls > 1)
      {
        // Compute a NullSpaceProjection
        if(!NullSpaceProjection(dqs, res.update_angles))
          {
            ROS_ERROR("Could not get null space projection");
            return false;
          }
      }
    
    // Add delta joint angles to current joint angles through fill.
    else
      res.update_angles = fill(dqs[0]);


	
    res.error = error;
    return true;
	
  }

}  //namespace controller


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Baxter_controller");

  //ROS_INFO("Main start");  
  
  ros::NodeHandle node("~");

  // Instantiate the controller
  force_controller::controller myControl(node);
  if(!myControl.start())
    {
      ROS_ERROR("Could not start controller, exiting");
      ros::shutdown();
      return 1;
    }
  ros::Duration(1.0).sleep();

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}  
