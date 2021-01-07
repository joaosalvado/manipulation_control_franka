#include<my_simple_controllers/state_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin


namespace my_simple_controllers {

void StateController::update(const ros::Time& time, const ros::Duration& period) {
   ROS_INFO("State Controller: here read joint handles, compute forward kinematic model and publish tool tip pose to TF");

   KDL::Tree mytree;
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
  
}

bool StateController::init(hardware_interface::JointStateInterface* hw, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {        
   ROS_INFO("State Controller: here read robot description from parameter server, initialize publishers, read parameters, load joint handles");


  
   /*ros::NodeHandle node;
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, std::string());
   if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }*/


  


   //Get all joint handle names
   auto handle_names = hw->getNames();
   
   for(auto handle_name : handle_names){
      this->joints_.push_back(hw->getHandle(handle_name));

   }
   return true;
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StateController,
                       controller_interface::ControllerBase)
