cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_biotac)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  sr_visualization_icons  
  std_msgs
  sr_robot_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS sr_robot_msgs std_msgs rospy rqt_gui rqt_gui_py

)
install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
install( FILES sr_change_controllers_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
