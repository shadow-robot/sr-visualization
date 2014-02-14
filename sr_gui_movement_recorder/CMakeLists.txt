cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_movement_recorder)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py sr_hand pr2_mechanism_msgs)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py sr_hand pr2_mechanism_msgs
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( DIRECTORY saved_src DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_gui_movement_recorder_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )