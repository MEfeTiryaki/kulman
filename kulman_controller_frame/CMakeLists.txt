cmake_minimum_required(VERSION 2.8.3)
project(kulman_controller_frame)

add_definitions("-std=c++11")

find_package(Boost REQUIRED COMPONENTS thread chrono)

find_package(catkin REQUIRED
  COMPONENTS
    kulman_msgs
    roscpp
    param_io
    geometry_msgs
    std_msgs
    sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    kulman_msgs
    roscpp
  	param_io
  	geometry_msgs
    std_msgs
    sensor_msgs
  DEPENDS
    Boost
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
