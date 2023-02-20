cmake_minimum_required(VERSION 3.0.2)
project(sr_health_check)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py )

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py 
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( FILES sr_health_check_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
