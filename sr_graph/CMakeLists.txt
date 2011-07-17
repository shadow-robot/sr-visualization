cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL TRUE)
SET(sr_graph_SOURCES src/main.cpp src/glwidget.cpp)

SET(sr_graph_HEADERS include/sr_graph/glwidget.h)
QT4_WRAP_CPP(sr_graph_HEADERS_MOC ${sr_graph_HEADERS})

#SET(sr_graph_FORMS ui/srvisualizer.ui)
#QT4_WRAP_UI(sr_graph_FORMS_HEADERS ${sr_graph_FORMS})

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

#EXECUTE_PROCESS(COMMAND "${PROJECT_SOURCE_DIR}/scripts/move_ui_headers.sh")

rosbuild_add_executable(sr_graph ${sr_graph_SOURCES} 
                                 ${sr_graph_HEADERS_MOC} )
#                                 ${sr_graph_FORMS_HEADERS})
TARGET_LINK_LIBRARIES(sr_graph ${QT_LIBRARIES})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
