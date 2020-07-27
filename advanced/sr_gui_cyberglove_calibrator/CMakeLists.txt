cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_cyberglove_calibrator)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py std_msgs std_srvs diagnostic_msgs sr_robot_msgs sr_visualization_icons)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py std_msgs std_srvs diagnostic_msgs sr_robot_msgs sr_visualization_icons
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis resource images DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_cyberglove_calibrator_plugin.xml sr_cyberglove_tweaker_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_cyberglove_calibrator.py)
endif()

