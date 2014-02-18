cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_change_muscle_controllers)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py pr2_mechanism_msgs sr_robot_msgs sr_visualization_icons)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py pr2_mechanism_msgs sr_robot_msgs sr_visualization_icons
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_change_controllers_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )