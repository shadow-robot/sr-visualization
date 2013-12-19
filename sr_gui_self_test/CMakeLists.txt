cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_self_test)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py diagnostic_msgs sr_robot_msgs)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py diagnostic_msgs sr_robot_msgs
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()