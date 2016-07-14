#include <sstream>
#include <ur5_hardware_interface/ur5_hardware_interface.h>

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
  //std::cout << "read:" << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
  {
    jnt_curr_pos_[i] = jnt_cmd_pos_[i];
    //std::cout << jnt_names_[i] << ": "<< jnt_curr_pos_[i] << std::endl;
  }

  //std::cout << std::endl; 
}

void Ur5HwInterface::write()
{
#if 1
  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
    std::cout << jnt_names_[i] << ": "<< jnt_cmd_pos_[i] << std::endl;

  std::cout << std::endl;
#endif
#if 0
  for(size_t i = 0; i< n_dof_; i++)
    RTT::log(Info) << "\n" << jnt_names_[i] << ": "<< jnt_cmd_pos_[i] << "\n"<< RTT::endlog(); 
#endif
}

ros::Time Ur5HwInterface::getTime() const 
{
    return ros::Time::now();
}

ros::Duration Ur5HwInterface::getPeriod() const 
{
    return ros::Duration(0.001);
}

