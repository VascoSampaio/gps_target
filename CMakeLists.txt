cmake_minimum_required(VERSION 2.8.3)
project(gps_target)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  multidrone_kml_parser
)

###########
## Build ##
###########

catkin_package(
  CATKIN_DEPENDS roscpp mavros_msgs multidrone_kml_parser
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
)

## Declare a C++ executable. With catkin_make all packages are built within a single CMake context. The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/gps_target_node.cpp)
add_executable(read_write_node src/read_write.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
   usbp-1
)

## Specify libraries to link a library or executable target against
target_link_libraries(read_write_node
  ${catkin_LIBRARIES}
   usbp-1
)