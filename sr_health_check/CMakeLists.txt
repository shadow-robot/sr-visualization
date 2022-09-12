cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_bootloader)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py )

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py 
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_health_check_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
