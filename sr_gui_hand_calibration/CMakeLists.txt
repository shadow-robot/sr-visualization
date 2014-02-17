cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_hand_calibration)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py sr_robot_msgs sr_robot_lib)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py sr_robot_msgs sr_robot_lib
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_gui_hand_calibration_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )