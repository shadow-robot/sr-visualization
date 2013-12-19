cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_grasp_controller)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py sr_hand)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py sr_hand
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()