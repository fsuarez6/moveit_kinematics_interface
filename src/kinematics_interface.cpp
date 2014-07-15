#include <moveit/kinematics_interface/kinematics_interface.h>

namespace moveit
{
  KinematicsInterface::KinematicsInterface() :
      nh_private_("~")
  {
    // Get parameters from the server
    nh_private_.param(std::string("planning_group"), planning_group_, std::string("arm"));
    if (!nh_private_.hasParam("planning_group"))
      ROS_WARN_STREAM("Parameter [~planning_group] not found, using default: " << planning_group_);
    // Get robot namespace
    robot_namespace_ = ros::this_node::getNamespace();
    setupRobotModel();
  }
  
  KinematicsInterface::~KinematicsInterface()
  {
  }
  
  void KinematicsInterface::getJointPositions(std::vector<double> &joint_positions)
  {
     kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_positions);
  }
  
  void KinematicsInterface::setJointPositions(std::vector<double>& joint_positions)
  {
     kinematic_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  }
  
  bool KinematicsInterface::setEndEffectorPose(const Eigen::Affine3d &pose, unsigned int attempts, double timeout)
  {
    // Here [attempts] is the number of random restart and [timeout] is the allowed time after each restart
    return kinematic_state_->setFromIK(joint_model_group_, pose, attempts, timeout);
  }
  
  bool KinematicsInterface::setEndEffectorPose(const geometry_msgs::Pose &pose, unsigned int attempts, double timeout)
  {
    // Here [attempts] is the number of random restart and [timeout] is the allowed time after each restart
    return kinematic_state_->setFromIK(joint_model_group_, pose, attempts, timeout);
  }
  
  bool KinematicsInterface::getManipulabilityIndex(double &manipulability_index) const
  {
    return kinematics_metrics_->getManipulability(*kinematic_state_, joint_model_group_, manipulability_index);  
  }
  
  bool KinematicsInterface::getManipulability(double &manipulability) const
  {
    return kinematics_metrics_->getManipulability(*kinematic_state_, joint_model_group_, manipulability);
  }
  
  void KinematicsInterface::setupRobotModel()
  {
    std::string kinematic_solver;
    if (ros::param::get(ros::this_node::getName() + "/" + planning_group_ + "/kinematics_solver", kinematic_solver))
      ROS_INFO_STREAM("\033[94m" << "Using solver: " << kinematic_solver << "\033[0m");
    else
    {
      ROS_ERROR_STREAM("Missing kinematic solver parameter: " << kinematic_solver);
      ros::shutdown();
      return;
    }
    // Get the RobotModel loaded from urdf and srdf files
    kinematic_model_ = rm_loader_.getModel();
    if (!kinematic_model_)
      ROS_ERROR("Could not load robot_description");
    else
      ROS_INFO_STREAM("robot_description loaded from: " << rm_loader_.getRobotDescription());
    // Get and print the name of the coordinate frame in which the transforms for this model are computed
    model_frame_ = kinematic_model_->getModelFrame();
    // Remove the slash from model_frame
    if (model_frame_.find("/") == 0)
      model_frame_.erase(0,1);
    ROS_INFO_STREAM("Model frame: " << model_frame_);
    // create a RobotState to keep track of the current robot pose
    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
    if (!kinematic_state_) {
      ROS_ERROR("Could not get RobotState from Model");
    }
    kinematic_state_->setToDefaultValues();
    // Setup the joint group
    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_);
    // Get the names of the controllable joints in the arm
    joint_names_ = joint_model_group_->getActiveJointModelNames();
    for(std::size_t i = 0; i < joint_names_.size(); ++i)
      ROS_DEBUG("Joint [%d]: %s", int(i), joint_names_[i].c_str());
    // Get the tip and base link
    base_link_ = "base_link";
    tip_link_ = joint_model_group_->getLinkModelNames().back();
    ROS_INFO_STREAM("Tip Link: " << tip_link_);
    // Get the joint limits
    typedef std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it_type;
    for(it_type it = rm_loader_.getURDF()->joints_.begin(); it != rm_loader_.getURDF()->joints_.end(); it++)
    {
      joint_limits_interface::getJointLimits(it->second, urdf_limits_[it->first]);
      if (!urdf_limits_[it->first].has_position_limits)
      {
        urdf_limits_[it->first].min_position = -M_PI;
        urdf_limits_[it->first].max_position = M_PI;
      }
      ROS_DEBUG_STREAM("Joint " << it->first << ": " << urdf_limits_[it->first].min_position << " " << urdf_limits_[it->first].max_position);
      ROS_DEBUG_STREAM("Joint " << it->first << " max_velocity: " << urdf_limits_[it->first].max_velocity);
    }    
    // Setup the kinematic metrics
    // Setup the kinematic metrics
    kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(kinematic_model_));
    /* Multiplier for JointLimitsPenalty.  Set penalty_multiplier to 0
    if you don't want this to have any effect on the metrics*/
    kinematics_metrics_->setPenaltyMultiplier(0.0);
  }  
} // namespace
