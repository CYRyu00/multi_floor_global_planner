cmake_minimum_required(VERSION 3.0.2)
project(multi_floor_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  message_filters
  tf2_ros
  costmap_2d
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_floor_planner
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs message_filters 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(a_star src/a_star.cpp)

add_executable(one2two_planner src/one2two_planner.cpp)
add_executable(one2three_planner src/one2three_planner.cpp)


target_link_libraries(one2two_planner
  
  a_star
  ${catkin_LIBRARIES}
)
target_link_libraries(one2three_planner
  
  a_star
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS scripts/initpose_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 
catkin_install_python(PROGRAMS scripts/goal_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 