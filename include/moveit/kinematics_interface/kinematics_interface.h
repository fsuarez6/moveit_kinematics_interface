#ifndef _MOVEIT_KINEMATICS_INTERFACE_H_
#define _MOVEIT_KINEMATICS_INTERFACE_H_

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <eigen_conversions/eigen_msg.h>

// Joint limits
#include <joint_limits_interface/joint_limits_urdf.h>

namespace moveit
{

  class KinematicsInterface
  {
    private:
      // Ros
      ros::NodeHandle                           nh_, nh_private_;
      // Kinematics
      robot_model_loader::RobotModelLoader      rm_loader_;
      robot_model::RobotModelPtr                kinematic_model_;
      robot_state::RobotStatePtr                kinematic_state_;
      robot_state::JointModelGroup*             joint_model_group_;
      kinematics_metrics::KinematicsMetricsPtr  kinematics_metrics_;
      // Joint limits
      std::map<std::string, joint_limits_interface::JointLimits> urdf_limits_;
      // Misc
      std::vector<std::string>                  joint_names_;
      std::string                               robot_namespace_;
      std::string                               planning_group_; 
      std::string                               model_frame_; 
      std::string                               tip_link_;
      std::string                               base_link_;
      
      void setupRobotModel();
    
    public:
      KinematicsInterface();
      ~KinematicsInterface();
      bool setEndEffectorPose(const Eigen::Affine3d &pose, unsigned int attempts = 0, double timeout = 0.0);
      bool setEndEffectorPose(const geometry_msgs::Pose &pose, unsigned int attempts = 0, double timeout = 0.0);
      void getJointPositions(std::vector<double> &joint_positions);
      void setJointPositions(std::vector<double>& joint_positions);
      bool getManipulabilityIndex(double &manipulability_index) const;
      bool getManipulability(double &manipulability) const;
      
      const std::vector<std::string>& getActiveJointModelNames() const { return joint_names_; };
      
      const std::string& getModelFrame() const { return model_frame_; };
      
      const std::map<std::string, joint_limits_interface::JointLimits>& getJointLimits() const 
      { return urdf_limits_; };
      
      const Eigen::Affine3d& getEndEffectorTransform()
      { return kinematic_state_->getGlobalLinkTransform(tip_link_); };
      
      const Eigen::Affine3d& getGlobalLinkTransform(const std::string &link_name)
      { return kinematic_state_->getGlobalLinkTransform(link_name); };
      
  };
  
  typedef boost::shared_ptr<moveit::KinematicsInterface> KinematicsInterfacePtr;
  typedef boost::shared_ptr<const moveit::KinematicsInterface> KinematicsInterfaceConstPtr;

} // namespace

#endif
