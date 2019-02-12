cmake_minimum_required(VERSION 2.8.3)
project(sr_data_visualization)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sr_robot_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sr_robot_msgs
  INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#add_executable(sr_moveit_run_benchmark src/SrRunBenchmark.cpp src/SrBenchmarkExecutor.cpp)
#target_link_libraries(sr_moveit_run_benchmark ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# if (CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     add_rostest(test/test_import_export_scenes.test)
#     add_rostest(test/test_import_export_queries.test)
# endif ()


