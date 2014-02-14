cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_motor_resetter)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py std_msgs std_srvs diagnostic_msgs)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py std_msgs std_srvs diagnostic_msgs
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_motor_resetter_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )