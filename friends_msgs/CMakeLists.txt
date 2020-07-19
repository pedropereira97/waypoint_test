cmake_minimum_required(VERSION 2.8.3)
project(friends_msgs)

find_package(catkin REQUIRED std_msgs geometry_msgs trajectory_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # ROS == Indigo.
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  # ROS > Indigo.
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS  std_msgs geometry_msgs trajectory_msgs
  CFG_EXTRAS export_flags.cmake
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)



