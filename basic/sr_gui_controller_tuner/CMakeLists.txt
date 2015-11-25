cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_controller_tuner)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py sr_robot_msgs controller_manager_msgs sr_visualization_icons)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py sr_robot_msgs controller_manager_msgs sr_visualization_icons
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( DIRECTORY data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_controller_tuner_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
