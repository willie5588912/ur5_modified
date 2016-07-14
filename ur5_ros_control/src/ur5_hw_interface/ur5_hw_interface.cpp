#if 0
#include <angles/angles.h>

#include <urdf_parser/urdf_parser.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <pluginlib/class_list_macros.h>

#include <rosbook_arm_hardware_gazebo/rosbook_arm_hardware_gazebo.h>
#endif

#include <sstream>
#include <ur5_hw_interface/ur5_hw_interface.h>

Ur5HwInterface::Ur5HwInterface():
    n_dof_(6)
{
  // Cleanup
  jnt_curr_pos_.clear();
  jnt_curr_vel_.clear();
  jnt_curr_eff_.clear();
  jnt_cmd_pos_.clear();
 
  // Joints
  jnt_names_.push_back("shoulder_pan_joint");
  jnt_names_.push_back("shoulder_lift_joint");
  jnt_names_.push_back("elbow_joint");
  jnt_names_.push_back("wrist_1_joint");
  jnt_names_.push_back("wrist_2_joint");
  jnt_names_.push_back("wrist_3_joint");
#if 0
  for (size_t i = 0; i < n_dof_; ++i)
  {
    std::stringstream ss;
    ss << "joint_" << i;
    jnt_names_.push_back(ss.str());
  }
#endif

  //Raw Data
  jnt_curr_pos_.resize(n_dof_);
  jnt_curr_vel_.resize(n_dof_);
  jnt_curr_eff_.resize(n_dof_);
  jnt_cmd_pos_.resize(n_dof_);

  // Hardware Interface
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(jnt_names_[i], &jnt_curr_pos_[i], &jnt_curr_vel_[i], &jnt_curr_eff_[i]));

    jnt_pos_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(jnt_names_[i]), &jnt_cmd_pos_[i]));

    ROS_DEBUG_STREAM("Registered joint '" << jnt_names_[i] << "' in the PositionJointInterface.");
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);

}

Ur5HwInterface::~Ur5HwInterface()
{
}

void Ur5HwInterface::read()
{
  std::cout << "read:" << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
  {
    jnt_curr_pos_[i] = jnt_cmd_pos_[i];
    std::cout << jnt_names_[i] << ": "<< jnt_curr_pos_[i] << std::endl;
  }

  std::cout << std::endl; 
}

void Ur5HwInterface::write()
{
  std::cout << "write:" << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
    std::cout << jnt_names_[i] << ": "<< jnt_cmd_pos_[i] << std::endl;

  std::cout << std::endl;
}

ros::Time Ur5HwInterface::getTime() const 
{
    return ros::Time::now();
}

ros::Duration Ur5HwInterface::getPeriod() const 
{
    return ros::Duration(0.1);
}
#if 0
  using namespace hardware_interface;

  ROSBookArmHardwareGazebo::ROSBookArmHardwareGazebo()
    : gazebo_ros_control::RobotHWSim()
  {}
#endif

#if 0
  bool ROSBookArmHardwareGazebo::initSim(const std::string& robot_namespace,
      ros::NodeHandle nh,
      gazebo::physics::ModelPtr model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    using gazebo::physics::JointPtr;
#endif

#if 0
    // Cleanup
    sim_joints_.clear();
    jnt_pos_.clear();
    jnt_vel_.clear();
    jnt_eff_.clear();
    jnt_pos_cmd_.clear();
#endif

#if 0
    // Simulation joints
    sim_joints_ = model->GetJoints();
    n_dof_ = sim_joints_.size();

    std::vector<std::string> jnt_names;
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_names.push_back(sim_joints_[i]->GetName());
    }
#endif

#if 0
    // Raw data
    jnt_pos_.resize(n_dof_);
    jnt_vel_.resize(n_dof_);
    jnt_eff_.resize(n_dof_);
    jnt_pos_cmd_.resize(n_dof_);
#endif

#if 0
    // Hardware interfaces
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_state_interface_.registerHandle(
          JointStateHandle(jnt_names[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));

      jnt_pos_cmd_interface_.registerHandle(
          JointHandle(jnt_state_interface_.getHandle(jnt_names[i]), &jnt_pos_cmd_[i]));

      ROS_DEBUG_STREAM("Registered joint '" << jnt_names[i] << "' in the PositionJointInterface.");
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_cmd_interface_);

#endif

#if 0
    // Position joint limits interface
    std::vector<std::string> cmd_handle_names = jnt_pos_cmd_interface_.getNames();
    for (size_t i = 0; i < n_dof_; ++i)
    {
      const std::string name = cmd_handle_names[i];
      JointHandle cmd_handle = jnt_pos_cmd_interface_.getHandle(name);

      using namespace joint_limits_interface;
      boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(name);
      JointLimits limits;
      SoftJointLimits soft_limits;
      if (!getJointLimits(urdf_joint, limits) || !getSoftJointLimits(urdf_joint, soft_limits))
      {
        ROS_WARN_STREAM("Joint limits won't be enforced for joint '" << name << "'.");
      }
      else
      {
        jnt_limits_interface_.registerHandle(
            PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));

        ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << name << "'.");
      }
    }

    // PID controllers
    pids_.resize(n_dof_);
    for (size_t i = 0; i < n_dof_; ++i)
    {
      ros::NodeHandle joint_nh(nh, "gains/" + jnt_names[i]);

      if (!pids_[i].init(joint_nh))
      {
        return false;
      }
    }

    return true;
  }

  void ROSBookArmHardwareGazebo::readSim(ros::Time time, ros::Duration period)
  {
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_pos_[i] += angles::shortest_angular_distance
          (jnt_pos_[i], sim_joints_[i]->GetAngle(0u).Radian());
      jnt_vel_[i] = sim_joints_[i]->GetVelocity(0u);
      jnt_eff_[i] = sim_joints_[i]->GetForce(0u);
    }
  }

  void ROSBookArmHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
  {
    // Enforce joint limits
    jnt_limits_interface_.enforceLimits(period);

    // Compute and send commands
    for (size_t i = 0; i < n_dof_; ++i)
    {
      const double error = jnt_pos_cmd_[i] - jnt_pos_[i];
      const double effort = pids_[i].computeCommand(error, period);

      sim_joints_[i]->SetForce(0u, effort);
    }
  }


PLUGINLIB_EXPORT_CLASS(rosbook_arm_hardware_gazebo::ROSBookArmHardwareGazebo, gazebo_ros_control::RobotHWSim)
#endif
