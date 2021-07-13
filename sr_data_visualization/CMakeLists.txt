cmake_minimum_required(VERSION 2.8.3)
project(sr_data_visualization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
  sr_robot_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    rqt_gui
    rqt_gui_py
    std_msgs
    sr_robot_msgs
)

install(DIRECTORY launch uis resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install( FILES sr_data_visualizer_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
