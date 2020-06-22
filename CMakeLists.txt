cmake_minimum_required(VERSION 2.8.3)
project(gps_target)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  multidrone_kml_parser
  message_generation
)



###########
## Build ##
###########

add_message_files(
  FILES
  SurveyGPS.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp mavros_msgs multidrone_kml_parser message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
)

## Declare a C++ executable. With catkin_make all packages are built within a single CMake context. The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/gps_target_node.cpp)


add_compile_options(-DDEBUG)
add_executable(${PROJECT_NAME}_node_debug src/gps_target_node.cpp)

add_executable(read_write_node src/read_write.cpp)




target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
   usbp-1
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node_debug
  ${catkin_LIBRARIES}
   usbp-1
)

## Specify libraries to link a library or executable target against
target_link_libraries(read_write_node
  ${catkin_LIBRARIES}
   usbp-1
)