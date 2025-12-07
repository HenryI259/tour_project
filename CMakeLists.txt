cmake_minimum_required(VERSION 3.0.2)
project(tour_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  kobuki_msgs
  actionlib
  move_base_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs kobuki_msgs actionlib move_base_msgs tf
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(tour src/tour.cpp)
target_link_libraries(tour ${catkin_LIBRARIES})
add_dependencies(tour ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(map src/map.cpp)
target_link_libraries(map ${catkin_LIBRARIES})
add_dependencies(map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
