#ifndef FORCE_CONTROLLER_H_
#define FORCE_CONTROLLER_H_
#include <cstring>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>

namespace force_controller 
{

  typedef struct INFO {
   std::vector<std::string> link_names;
   std::vector<std::string> joint_names;
  } KinematicSolverInfo;

  class Kinematics 
  {
	public:
	  Kinematics();

	  bool init(std::string tip_name, int &no_jts);

	  typedef boost::shared_ptr<Kinematics> Ptr;
	
	  static Ptr create(std::string tip_name, int &no_jts)
	  {
	    Ptr parm_kinematics = Ptr(new Kinematics());
	    if (parm_kinematics->init(tip_name, no_jts))
		{
	      ROS_INFO_ONCE("All init finished for tip: %s", tip_name.c_str());
	      return parm_kinematics;
	    }
	    else
	      ROS_ERROR("Couldn't initialize for tip: %s", tip_name.c_str());
	    return Ptr();
	  }
      bool readJoints(urdf::Model &robot_model);

      inline int getJointIndex(const std::string &name)
	  {
		for (unsigned int i = 0; i < info_.joint_names.size(); i++)
		{
		  if (info_.joint_names[i] == name)
			return i;
  		}
		return -1;
	  }

    // Compute the Jacobian. 
    // Input: Joint Angles, Joint Names, and 
    // Output: Jacobian Matrix output result.  
	  bool getJacobian(std::vector<double> joints, std::vector<std::string> names, Eigen::MatrixXd& jacobian);
	  bool getJacPseudoInv(std::vector<double> joints, std::vector<std::string> names, Eigen::MatrixXd& jacobian);

	 private:
	  ros::NodeHandle nh_, nh_private_;
	  std::string root_name_, tip_name_;
	  KDL::Chain chain_;
	  unsigned int num_joints_;
      KinematicSolverInfo info_;

	  bool loadModel(const std::string xml);

  };

}
#endif
