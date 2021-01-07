#include<my_simple_controllers/standstill_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

namespace my_simple_controllers {

void StandStillController::update(const ros::Time& time, const ros::Duration& period) {
   
   for(int joint_id = 0; joint_id < joints_.size(); ++joint_id){
       auto curr_joint_pos = joints_[joint_id].getPosition();
       auto curr_joint_vel = joints_[joint_id].getVelocity();

       int kp = 100;
       int kd = 10;
       auto curr_error = initial_joint_config_[joint_id] - curr_joint_pos;
       //double control_effort = kp*curr_error - kd*curr_joint_vel;
       
       double control_effort = kp*curr_error + kd*(curr_error - prev_error_[joint_id])*(1/period.toSec());
       joints_[joint_id].setCommand(control_effort);
       this->prev_error_[joint_id] = curr_error; 
       //Printing the error  
        ROS_INFO("Error joint_%d  error: %f",joint_id, curr_error);
        
  
   }
  
   
}

bool StandStillController::init(hardware_interface::EffortJointInterface* hw, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {        
   ROS_INFO("State Controller: here read robot description from parameter server, initialize publishers, read parameters, load joint handles");

   //Get all joint handle names
   auto handle_names = hw->getNames();

   for(auto handle_name : handle_names){
      this->joints_.push_back(hw->getHandle(handle_name));
   }
   return true;
}

void StandStillController::starting(const ros::Time& time) {
    for(auto joint_ : joints_){
        initial_joint_config_.push_back(joint_.getPosition());
    }

    this->prev_error_ = std::vector<double> (joints_.size(),0);
}


}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StandStillController,
                       controller_interface::ControllerBase)
