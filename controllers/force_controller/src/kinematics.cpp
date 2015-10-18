#include <cstring>
#include <ros/ros.h>
#include <force_controller/kinematics.h>

namespace force_controller 
{

  Kinematics::Kinematics(): nh_private_("~") 
  {
  }

  bool Kinematics::init(std::string tip, int &no_jts)
  {
    // Get URDF XML
	std::string urdf_xml, full_urdf_xml;
	tip_name_ = tip;
	nh_.param("urdf_xml", urdf_xml, std::string("robot_description"));
	nh_.searchParam(urdf_xml, full_urdf_xml);
	ROS_DEBUG("Reading xml file from parameter server");
	std::string result;
	if (!nh_.getParam(full_urdf_xml, result)) 
	{
	  ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
	  return false;
    }

	if (!nh_.getParam("root_name", root_name_)) 
	{
	  ROS_FATAL("GenericIK: No root name found on parameter server");
	  return false;
	}

	if (!loadModel(result)) 
	{
	  ROS_FATAL("Could not load models!");
	  return false;
	}

    no_jts=num_joints_;
    return true;
  }


  bool Kinematics::loadModel(const std::string xml)
  {
    urdf::Model robot_model;
    KDL::Tree tree;
    if (!robot_model.initString(xml)) 
    {
      ROS_FATAL("Could not initialize robot model");
      return -1;
    }
    if (!kdl_parser::treeFromString(xml, tree)) 
    {
      ROS_ERROR("Could not initialize tree object");
      return false;
    }
    if (!tree.getChain(root_name_, tip_name_, chain_)) 
    {
	  ROS_ERROR("Could not initialize chain object for root_name %s and tip_name %s",root_name_.c_str(), tip_name_.c_str());
	  return false;
	}
	if (!readJoints(robot_model)) 
	{
	  ROS_FATAL("Could not read information about the joints");
	  return false;
	}
    return true;
  }

  bool Kinematics::readJoints(urdf::Model &robot_model) 
  {
	num_joints_ = 0;
	boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name_);
	boost::shared_ptr<const urdf::Joint> joint;
	for (int i = 0; i < chain_.getNrOfSegments(); i++)
    while (link && link->name != root_name_) 
	{
      if (!(link->parent_joint)) 
        break;
      
      joint = robot_model.getJoint(link->parent_joint->name);
      if (!joint) 
	  {
        ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
        return false;
      }
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) 
        num_joints_++;

      link = robot_model.getLink(link->getParent()->name);
    }
	info_.joint_names.resize(num_joints_);
	info_.link_names.resize(num_joints_);

	link = robot_model.getLink(tip_name_);
	unsigned int i = 0;
	while (link && i < num_joints_) 
	{
	  ROS_INFO_ONCE("Number of Joints: %d, for tip: %s", num_joints_, tip_name_.c_str());
	  joint = robot_model.getJoint(link->parent_joint->name);
	  if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) 
	  {
		int index = num_joints_ - i - 1;
		info_.joint_names[index] = joint->name;
		info_.link_names[index] = link->name;
		i++;
	  }
      link = robot_model.getLink(link->getParent()->name);
  	}
	return true;
  }

  bool Kinematics::getJacobian(std::vector<double> joints, std::vector<std::string> names, Eigen::MatrixXd& jacobian)
  {
	KDL::Jacobian J(7);  
	KDL::JntArray jntPos;
	KDL::ChainJntToJacSolver j_solver(chain_);
    jntPos.resize(joints.size());

	//ROS_INFO_STREAM_ONCE("joint size: " << joints.size());
    for (unsigned int j = 0; j < names.size(); j++) 
	{
      for (unsigned int i = 0; i < num_joints_; i++) 
	  {
        if (names[j] == info_.joint_names[i])
		{
          jntPos(i) = joints[j];
          break;
        } 
      }
    }
	//ROS_WARN("Before calling Joint to Jac");
	j_solver.JntToJac(jntPos, J);
	int row, col;
	row = J.rows();
	col = J.columns();
	//ROS_WARN_STREAM_ONCE("Jac size: rows: " << row << ", cols: " << col);
	jacobian.resize(row, col);
	jacobian.setZero(row, col);
	for(unsigned int i=0; i<row; i++)
	{
	  for(unsigned int j=0; j<col; j++)
		jacobian(i, j) = J(i, j);
	}
	return true;
  }

  bool Kinematics::getJacPseudoInv(std::vector<double> joints, std::vector<std::string> names, Eigen::MatrixXd& jacobian)
  {
    Eigen::MatrixXd j, jt, temp, inv;

	getJacobian(joints, names, j);
	jt = j.transpose();
	temp = j*jt;
	inv = temp.inverse();
	jacobian = jt*inv;
	return true;
  }
}  //namespace

