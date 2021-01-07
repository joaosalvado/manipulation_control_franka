# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3".split(';') if "${prefix}/include;/usr/local/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;kdl_parser;roscpp;visualization_msgs;tf2;tf2_ros;eigen_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmy_simple_controllers".split(';') if "-lmy_simple_controllers" != "" else []
PROJECT_NAME = "my_simple_controllers"
PROJECT_SPACE_DIR = "/home/ohmy/js_ws/github_joao/manipulation_control_franka/install"
PROJECT_VERSION = "0.0.0"
