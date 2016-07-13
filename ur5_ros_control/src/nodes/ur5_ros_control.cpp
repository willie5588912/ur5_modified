#include <ur5_hw_interface/ur5_hw_interface.h>


int main(int argc, char **argv)
{
  //initialize ros
  ros::init(argc, argv, "ur5_ros_control_node");
  ros::NodeHandle nh;

  Ur5HwInterface robot;
  controller_manager::ControllerManager cm(&robot, nh);

  //start loop
  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
     robot.read();
     cm.update(robot.getTime(), robot.getPeriod());
     robot.write();
	 
	 rate.sleep();
  }
  spinner.stop();

  return 0;
}
