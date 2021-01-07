#include<my_simple_controllers/torque_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin


namespace my_simple_controllers {

void TorqueController::update(const ros::Time& time, const ros::Duration& period) {
    int kp = 100;
    int kd = 10;
   std::vector<double> q, q_dot, q_dot_dot, c, u;
   for(int joint_id = 0; joint_id < joints_.size(); ++joint_id){
       u.push_back(0.0);
       q.push_back( joints_[joint_id].getPosition() );
       //q_dot.push_back( (q[joint_id] - q_prev[joint_id])*(1/period.toSec()) );
       q_dot.push_back(joints_[joint_id].getVelocity());
       q_dot_dot.push_back( (q_dot[joint_id] - q_dot_prev[joint_id])*(1/period.toSec()) );
    
        q_d[0] =  cos(1*ros::Time::now().toSec());
        q_d[3] =  cos(2*ros::Time::now().toSec());
       auto curr_error = q_d[joint_id] - q[joint_id];
       auto curr_error_dot = q_dot_d[joint_id] - q_dot[joint_id];
       //c.push_back(  kp*curr_error - kd*q_dot[joint_id]);
       //c.push_back(   kp*curr_error + kd*(curr_error - prev_error_[joint_id])*(1/period.toSec()) );
       c.push_back( q_dot_dot_d[joint_id] +  kp*curr_error + kd*(curr_error - prev_error_[joint_id])*(1/period.toSec()) );
       //c.push_back( q_dot_dot_d[joint_id] +  kp*curr_error + kd*curr_error_dot );

        ROS_INFO("Error joint_%d  error: %f",joint_id, curr_error);

       //Update prev
       prev_error_[joint_id] = curr_error;
       q_prev[joint_id] = q[joint_id];
       q_dot_prev[joint_id] = q_dot_dot[joint_id];
   }
  
   this->NE(q_d, q_dot_d, q_dot_dot_d, u);
   //this->NE(q, q_dot, c, u);
   //
   //Printing the computed controls
    std::for_each(u.begin(), u.end(), [](double u_i){
           std::cout << " " << u_i << " ";

       });
       std::cout << std::endl;

   for(int joint_id = 0; joint_id < joints_.size(); ++joint_id){
              joints_[joint_id].setCommand(u[joint_id] + c[joint_id]);
              //joints_[joint_id].setCommand(u[joint_id]);
   }
       
    
        //Printing the error  
  
   
  
   
}

bool TorqueController::init(hardware_interface::EffortJointInterface* hw, 
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
   this->rotational_matrices_.resize(this->joints_.size()); 
   this->frame_translations_.resize(this->joints_.size());

   //Initilize Newton-Euler params
   ne0.w_0 = Eigen::Vector3d::Zero(3); ne0.w_dot_0 = Eigen::Vector3d::Zero(3); ne0.a_0 = Eigen::Vector3d::Zero(3);
   ne0.f_ext = Eigen::Vector3d::Zero(3); ne0.t_ext= Eigen::Vector3d::Zero(3);
   ne0.g_0 = Eigen::Vector3d(0, 0, 10); 
   //Dummy Initialization 
   dummy_init(root_nh);
  

   return true;
}

void TorqueController::starting(const ros::Time& time) {
    for(auto joint_ : joints_){
        q_d.push_back(joint_.getPosition()); 
        q_dot_d.push_back(0.0); 
        q_dot_dot_d.push_back(0.0);

        q_prev.push_back(joint_.getPosition());
        q_dot_prev.push_back(joint_.getVelocity());
    }

    this->prev_error_ = std::vector<double> (joints_.size(),0);
}

void TorqueController::AR(
    const Eigen::Vector3d &w_prev, const Eigen::Vector3d &w_dot_prev, const Eigen::Vector3d &a_prev,
    const double q, const double q_dot, const double q_dot_dot, std::size_t link_id,
    Eigen::Vector3d &w_next, Eigen::Vector3d &w_dot_next, Eigen::Vector3d &a_next){  
        Eigen::Vector3d z_prev(0,0,1);
        //Angular  
        w_next = rotational_matrices_[link_id].transpose() * ( w_prev + (q_dot*z_prev));
        w_dot_next = rotational_matrices_[link_id].transpose() * ( w_dot_prev + q_dot_dot*z_prev + q_dot*w_prev.cross(z_prev));
        //Linear Acceleration
        a_next = rotational_matrices_[link_id].transpose()*a_prev + w_dot_next.cross(frame_translations_[link_id])
                        + w_next.cross( w_next.cross(frame_translations_[link_id]) );
}

void TorqueController::FP(const Eigen::Vector3d &t, const double q_dot,
                          double &u, int link_id){
    Eigen::Vector3d z_prev(0,0,1);
    u = t.transpose()*this->rotational_matrices_[link_id].transpose()*z_prev;
}


void TorqueController::FTR(
    const Eigen::Vector3d &w, const Eigen::Vector3d &w_dot, const Eigen::Vector3d &a,
    const Eigen::Vector3d &f_next, const Eigen::Vector3d &t_next, std::size_t link_id,
    Eigen::Vector3d &f, Eigen::Vector3d &t){
        //Acceleration of link center of mass
        auto a_cm = a + w_dot.cross(dynamic_params_[link_id].r_cm) + w.cross(w.cross(dynamic_params_[link_id].r_cm));
        //Force
        f = rotational_matrices_[link_id]*f_next + dynamic_params_[link_id].mass*a_cm;
        //f = f_next + dynamic_params_[link_id].mass*a_cm;
        //Torque
        t = rotational_matrices_[link_id]*t_next - 
            f.cross( frame_translations_[link_id]+dynamic_params_[link_id].r_cm ) + 
            rotational_matrices_[link_id]*f_next.cross( dynamic_params_[link_id].r_cm ) +
            dynamic_params_[link_id].inertia*w_dot +
            w.cross( dynamic_params_[link_id].inertia*w );
        /*t = t_next - 
            f.cross( frame_translations_[link_id]+dynamic_params_[link_id].r_cm ) + 
            rotational_matrices_[link_id]*f_next.cross( dynamic_params_[link_id].r_cm ) +
            dynamic_params_[link_id].inertia*w_dot +
            w.cross( dynamic_params_[link_id].inertia*w );*/


}


void TorqueController::NE(
    const std::vector<double> &q, 
    const std::vector<double> &q_dot, 
    const std::vector<double> &q_dot_dot,
    std::vector<double> &u){
        //Forward Acceleration Recursion
        std::vector<Eigen::Vector3d> w(num_links_+1), w_dot(num_links_+1), a(num_links_+1);
        w[0] = ne0.w_0;
        w_dot[0] = ne0.w_dot_0;
        a[0] = ne0.a_0 - ne0.g_0;
        for(int link_id = 0; link_id < num_links_; ++link_id){
            AR(w[link_id], w_dot[link_id], a[link_id], 
               q[link_id], q_dot[link_id], q_dot_dot[link_id],
               link_id, w[link_id+1],  w_dot[link_id+1], a[link_id+1]);
        }

        //Backward Torque/Force Recursion
        std::vector<Eigen::Vector3d> f(num_links_+1),t(num_links_+1);
        f[num_links_] = ne0.f_ext;
        t[num_links_] = ne0.t_ext;
        for(int link_id = num_links_; link_id > 0; --link_id){
            FTR(w[link_id], w_dot[link_id], a[link_id],
                f[link_id], t[link_id], link_id-1, 
                f[link_id-1], t[link_id-1]);
        }

        //Forward Propagation from torque to control
        for(int link_id = 0; link_id < num_links_; ++link_id){
            this->FP(t[link_id], q_dot[link_id], u[link_id], link_id);
        } 

}
    

void TorqueController::dummy_init(ros::NodeHandle &root_nh){

    //Reading Kld tree from robot_description
    KDL::Tree my_tree;
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    
    //Get kinematic open chain of the arm 
    KDL::Chain chain;
    my_tree.getChain("panda_link0", "panda_link7", chain);
    std::cout << "Nr Segments: " << chain.getNrOfSegments() << std::endl;

    
    for(int link_id = 0; link_id < chain.getNrOfSegments(); ++link_id){
        std::cout << "Link id: " << link_id << std::endl; 
        //Get Frame translations and rotational matrices
        KDL::Frame frame = chain.getSegment(link_id).getFrameToTip();
        KDL::Rotation R = frame.M;
        KDL::Vector p = frame.p;
        this->frame_translations_[link_id]  = kdl_conv::toEigen(p);
        this->rotational_matrices_[link_id] = kdl_conv::toEigenMatrix(R);
        std::cout<< "i^r_(i,i-1) :" << this->frame_translations_[link_id] << std::endl;
        std::cout<< "i^R_(i,i-1) :" << this->rotational_matrices_[link_id] << std::endl;

        //Get dynamic parameters
        KDL::RigidBodyInertia rb= chain.getSegment(link_id).getInertia();
        this->dynamic_params_[link_id].mass = rb.getMass();
        this->dynamic_params_[link_id].r_cm = kdl_conv::toEigen( rb.getCOG() );
        this->dynamic_params_[link_id].inertia = Eigen::Map<const Eigen::Matrix3d>(rb.getRotationalInertia().data);
        std::cout<< "i^r_(i,c_i) :" << this->dynamic_params_[link_id].r_cm  << std::endl;
        

    }
    //exit(1);
   
}



  


}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::TorqueController,
                       controller_interface::ControllerBase)
