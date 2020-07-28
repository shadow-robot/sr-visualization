cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_shadow_glove_calibration)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_gui_shadow_glove_calibration.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
