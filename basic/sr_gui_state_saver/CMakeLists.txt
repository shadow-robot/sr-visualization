cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_state_saver)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py sr_robot_commander sr_visualization_icons)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py sr_visualization_icons sr_robot_commander
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_state_saver.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
