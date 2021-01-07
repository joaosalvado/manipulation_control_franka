#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <eigen3/Eigen/Dense>

#include <kdl_parser/kdl_parser.hpp>
#include <eigen3/Eigen/Geometry>
#include <kdl/frames.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <my_simple_controllers/kdl_conv.h>

#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


namespace my_simple_controllers
{
    typedef struct link_params
        {
            double mass;
            Eigen::Vector3d r_cm;    //vector between O_i and center of Mass of link i in i-frame coordinates
            Eigen::Matrix3d inertia; //link inertial matrix
        } link_params;

        typedef struct ne_params
        {
            Eigen::Vector3d g_0;
            Eigen::Vector3d w_0;
            Eigen::Vector3d w_dot_0;
            Eigen::Vector3d a_0;
            Eigen::Vector3d f_ext;
            Eigen::Vector3d t_ext;
        } ne_params;
        
    class TorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        

        //method executed by controller manager on every loop when controller is running
        //NOTE: MUST BE implemented
        virtual void update(const ros::Time &time, const ros::Duration &period) override;

        //initialize controller with access to hardware interface and node handles
        virtual bool init(hardware_interface::EffortJointInterface *hw,
                          ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        virtual void starting(const ros::Time &time) override;

    private:
        void dummy_init(ros::NodeHandle &root_nh);
  
        void AR(const Eigen::Vector3d &w_prev, const Eigen::Vector3d &w_dot_prev, const Eigen::Vector3d &a_prev,
                const double q, const double q_dot, const double q_dot_dot, std::size_t link_id,
                Eigen::Vector3d &w_next, Eigen::Vector3d &w_dot_next, Eigen::Vector3d &a_next);

        void FTR(const Eigen::Vector3d &w, const Eigen::Vector3d &w_dot, const Eigen::Vector3d &a,
                 const Eigen::Vector3d &f_next, const Eigen::Vector3d &t_next, std::size_t link_id,
                 Eigen::Vector3d &f, Eigen::Vector3d &t);

        void NE(const std::vector<double> &q,
                const std::vector<double> &q_dot,
                const std::vector<double> &q_dot_dot,
                std::vector<double> &u);
        
        void FP(const Eigen::Vector3d &t, const double q_dot,
                double &u, int link_id);

        std::size_t num_links_;
        std::vector<hardware_interface::JointHandle> joints_;
        std::vector<double> q_d, q_dot_d, q_dot_dot_d;
        std::vector<double> q_prev, q_dot_prev;
        std::vector<double> prev_error_;

        std::vector<link_params> dynamic_params_;          //dynamic param per link
        std::vector<Eigen::Matrix3d> rotational_matrices_; //rotational_matrices_[i] = (i-1)^R_i , between (i-1)-frame and i-frame
        std::vector<Eigen::Vector3d> frame_translations_;  //frame_translations_[i] = (i-1)^r_i, translation between (i-1)-frame and i-frame

        ne_params ne0;


    };

}; // namespace my_simple_controllers

#endif
