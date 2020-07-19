cmake_minimum_required(VERSION 2.8.3)
project(waypoint_navigator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs friends_msgs geodetic_utils glog_catkin)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

add_executable(waypoint_navigator_node src/waypoint_navigator_node.cpp)

target_link_libraries(waypoint_navigator_node ${catkin_LIBRARIES})

target_link_libraries(waypoint_navigator_node ${JSONCPP_LIBRARIES})
