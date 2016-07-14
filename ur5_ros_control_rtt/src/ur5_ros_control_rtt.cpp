#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <boost/thread.hpp>

#include <ur5_hardware_interface/ur5_hardware_interface.h>
#include <controller_manager/controller_manager.h>


using namespace RTT;

class UR5RosControl : public RTT::TaskContext{
  private:
    // Necessary components to run thread for serving ROS callbacks
    boost::thread non_rt_ros_queue_thread_;
    boost::shared_ptr<ros::NodeHandle> non_rt_ros_nh_;
    ros::CallbackQueue non_rt_ros_queue_;

    // The (example) hardware interface
    boost::shared_ptr<Ur5HwInterface> hw_interface_;

    // The controller manager
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // For saving last update time, so period can be handed to controller manager
    ros::Time last_update_time_;

  public:
    UR5RosControl(const std::string& name):
      TaskContext(name)
    {}

    ~UR5RosControl()
    {}

  private:

    bool configureHook(){
      non_rt_ros_nh_.reset(new ros::NodeHandle(""));
      non_rt_ros_nh_->setCallbackQueue(&non_rt_ros_queue_);
      this->non_rt_ros_queue_thread_ = boost::thread( boost::bind( &UR5RosControl::serviceNonRtRosQueue,this ) );

      hw_interface_.reset(new Ur5HwInterface);

      controller_manager_.reset(new controller_manager::ControllerManager(hw_interface_.get(), *non_rt_ros_nh_));

      last_update_time_ = rtt_rosclock::rtt_now();

      return true;
    }

    void updateHook(){

      // Get current system time (for timestamps of ROS messages)
      ros::Time now (rtt_rosclock::host_now());

      // Get guaranteed monotonic time for period computation
      ros::Time now_monotonic(rtt_rosclock::rtt_now());

      ros::Duration period (now_monotonic - last_update_time_);
      last_update_time_ = now_monotonic;

      hw_interface_->read();
      controller_manager_->update(now, period);
      hw_interface_->write();

      log(Info) << "UR5Control Update !" <<endlog();
    }

    void cleanupHook(){
      non_rt_ros_nh_->shutdown();
      non_rt_ros_queue_thread_.join();
    }

    void serviceNonRtRosQueue()
    {
      static const double timeout = 0.001;

      while (this->non_rt_ros_nh_->ok()){
        this->non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
      }
    }
};
ORO_CREATE_COMPONENT(UR5RosControl)
