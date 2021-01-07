#ifndef SIMPLE_STATE_CONTROLLER_H
#define SIMPLE_STATE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <eigen3/Eigen/Dense>



namespace my_simple_controllers {

  class StateController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
    public:
      //method executed by controller manager on every loop when controller is running
      //NOTE: MUST BE implemented
      virtual void update(const ros::Time& time, const ros::Duration& period);

      //initialize controller with access to hardware interface and node handles
      virtual bool init(hardware_interface::JointStateInterface* hw, 
		      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    private:
    std::vector<hardware_interface::JointStateHandle> joints_;
  };


};

#endif
