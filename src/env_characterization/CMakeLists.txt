cmake_minimum_required(VERSION 2.8.3)
project(env_characterization)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  message_generation
  octomap_ros
  tf
  geometry_msgs
  sensor_msgs
  cv_bridge
  octomap_msgs
  filters
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(octomap REQUIRED)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   classify_map.srv
   action_srv.srv
 )
 
 add_message_files(
   FILES
   PathNode.msg
   PathNodeArray.msg
   Action.msg
   Feature.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   nav_msgs
   grid_map_msgs
   env_characterization
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(
  example_generation
  src/example_generation.cpp
)

add_executable(
  env_characterization
  src/env_characterization.cpp
)

add_executable(
  sim_handler
  src/sim_handler.cpp
)

add_executable(
  gazebo_tf
  src/gazebo_tf.cpp
)

add_executable(
  navigation
  src/navigation.cpp
)

add_library(snake_control SHARED src/snake_control.cc)
target_link_libraries(snake_control ${GAZEBO_LIBRARIES})

add_library(car_control SHARED src/car_control.cc)
target_link_libraries(car_control ${GAZEBO_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(
  example_generation
  ${catkin_LIBRARIES}
)

target_link_libraries(
  env_characterization
  ${catkin_LIBRARIES}
)

target_link_libraries(
  sim_handler
  ${catkin_LIBRARIES}
)

target_link_libraries(
  gazebo_tf
  ${catkin_LIBRARIES}
)

target_link_libraries(
  navigation
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
