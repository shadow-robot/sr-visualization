cmake_minimum_required(VERSION 2.8.3)
project(sr_visualization_icons)
find_package(catkin REQUIRED)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

install( DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )