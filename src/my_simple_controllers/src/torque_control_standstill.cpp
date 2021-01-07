#include<my_simple_controllers/torque_control_standstill.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin


namespace my_simple_controllers {

void TorqueControlStandstill::update(const ros::Time& time, const ros::Duration& period) {
    int kp = 100;
    int kd = 10;

   

   for(int joint_id = 0; joint_id < joints_.size(); ++joint_id){
       u(joint_id) = 0.0;
       q(joint_id) = joints_[joint_id].getPosition();
       q_dot(joint_id) = ( q(joint_id) - q_prev(joint_id) ) * ( 1/period.toSec() );
       q_dot_dot(joint_id) = ( q_dot(joint_id) - q_dot_prev(joint_id) )*( 1/period.toSec() );
    
        
       auto curr_error = q_d(joint_id) - q(joint_id);
       auto curr_error_dot = q_dot_d(joint_id) - q_dot(joint_id);
       //double control_effort = kp*curr_error - kd*curr_joint_vel;
       c(joint_id) =  q_dot_dot_d(joint_id) +  kp*curr_error 
                    + kd*( curr_error - prev_error_(joint_id) ) * ( 1/period.toSec() );
       //c.push_back( q_dot_dot_d[joint_id] +  kp*curr_error + kd*curr_error_dot );

        ROS_INFO("Error joint_%d  error: %f",joint_id, curr_error);

       //Update prev
       prev_error_(joint_id) = curr_error;
       q_prev(joint_id) = q(joint_id);
       q_dot_prev(joint_id) = q_dot_dot(joint_id);
   }
  
  
    auto NE = KDL::ChainIdSolver_RNE(chain, kdl_conv::toKdlVector(ne0.g_0));
  
    auto ne_success = NE.CartToJnt(q, q_dot, c, f_ext, u);

    if(ne_success >=0){
        std::cout << u.data <<std::endl;
        printf("%s \n","ID3 -> Succes, thanks KDL!");
    }else{
        printf("%s \n","ID3 ->Error: could not calculate Newton Euler");
    }


   for(int joint_id = 0; joint_id < joints_.size(); ++joint_id){
              joints_[joint_id].setCommand(u(joint_id));
   }
       
    
        //Printing the error  
  
   
  
   
}

bool TorqueControlStandstill::init(hardware_interface::EffortJointInterface* hw, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {        
   ROS_INFO("Torque Controller: Initializing joints and dynamic params");

   //Get all joint handle names
   auto handle_names = hw->getNames();

   for(auto handle_name : handle_names){
      this->joints_.push_back(hw->getHandle(handle_name));
   }
   
   this->num_links_ = this->joints_.size();
   std::cout << "Nr Links: " << num_links_;
   this->dynamic_params_.resize(this->joints_.size()); 


   //Initilize Newton-Euler params
   ne0.w_0 = Eigen::Vector3d::Zero(3); ne0.w_dot_0 = Eigen::Vector3d::Zero(3); ne0.a_0 = Eigen::Vector3d::Zero(3);
   ne0.f_ext = Eigen::Vector3d::Zero(3); ne0.t_ext= Eigen::Vector3d::Zero(3);
   ne0.g_0 = Eigen::Vector3d(0, 0, -8); 
   //Dummy Initialization 
   dummy_init(root_nh);
  
   

   return true;
}

void TorqueControlStandstill::starting(const ros::Time& time) {
    q_d = KDL::JntArray(joints_.size());
    q_dot_d = KDL::JntArray(joints_.size());
    q_dot_dot_d = KDL::JntArray(joints_.size());

    q_prev = KDL::JntArray(joints_.size());
    q_dot_prev  = KDL::JntArray(joints_.size());
    prev_error_ = KDL::JntArray(joints_.size());
    
    q = KDL::JntArray(joints_.size());
    q_dot = KDL::JntArray(joints_.size());
    q_dot_dot = KDL::JntArray(joints_.size());
    c = KDL::JntArray(joints_.size());
    u = KDL::JntArray(joints_.size());

    f_ext = KDL::Wrenches();

    for(int joint_id = 0; joint_id < joints_.size(); ++joint_id ){
        //q_d(joint_id) = joints_[joint_id].getPosition(); 
        q_d(joint_id) = 0; 
        q_dot_d(joint_id) = 0.01; 
        q_dot_dot_d(joint_id) = 0.01;

        q_prev(joint_id) = joints_[joint_id].getPosition(); 
        q_dot_prev(joint_id) = joints_[joint_id].getVelocity(); 
        prev_error_ (joint_id) = 0.0;


        
        f_ext.push_back(KDL::Wrench().Zero());
    }



}



void TorqueControlStandstill::dummy_init(ros::NodeHandle &root_nh){

    //Reading Kld tree from robot_description
    KDL::Tree my_tree;
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    } else{
        std::cerr << "number of joint: " << my_tree.getNrOfJoints() << std::endl;
        std::cerr << "number of links: " << my_tree.getNrOfSegments() << std::endl;
    }


    //Get kinematic open chain of the arm 
    my_tree.getChain("panda_link0", "panda_link7", chain);
    
    
    std::cout << "Nr Segments: " << chain.getNrOfSegments() << std::endl;
    std::cout << "Nr Joints: " << chain.getNrOfJoints() << std::endl;

    
}



  


}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::TorqueControlStandstill,
                       controller_interface::ControllerBase)
