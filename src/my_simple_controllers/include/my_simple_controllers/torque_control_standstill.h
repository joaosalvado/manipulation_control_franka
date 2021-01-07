#ifndef TORQUE_CONTROL_STANDSTILL_H
#define TORQUE_CONTROL_STANDSTILL_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <eigen3/Eigen/Dense>

#include <kdl_parser/kdl_parser.hpp>
#include <eigen3/Eigen/Geometry>
#include <kdl/frames.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <my_simple_controllers/kdl_conv.h>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>


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
        
    class TorqueControlStandstill : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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

        std::size_t num_links_;
        std::vector<hardware_interface::JointHandle> joints_;
        KDL::JntArray q, q_dot, q_dot_dot, c, u;
        KDL::JntArray q_d, q_dot_d, q_dot_dot_d;
        KDL::JntArray q_prev, q_dot_prev;
        KDL::JntArray prev_error_;
        KDL::Wrenches f_ext;

        KDL::Chain chain;
        //KDL::ChainIdSolver_RNE NE;
        ne_params ne0;
        std::vector<link_params> dynamic_params_;          //dynamic param per link

    };

}; // namespace my_simple_controllers

#endif
