#ifndef STAND_STILL_CONTROLLER_H
#define STAND_STILL_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace my_simple_controllers {

  class StandStillController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
      //method executed by controller manager on every loop when controller is running
      //NOTE: MUST BE implemented
      virtual void update(const ros::Time& time, const ros::Duration& period)override;

      //initialize controller with access to hardware interface and node handles
      virtual bool init(hardware_interface::EffortJointInterface* hw, 
		      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

      virtual void starting(const ros::Time& time) override;

    private:  
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<double> initial_joint_config_;
    std::vector<double> prev_error_;

  };


};

#endif
