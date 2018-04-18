cmake_minimum_required(VERSION 2.8.3)
project(kulman_joystick)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  arac_model
  param_io
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
)


find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    arac_model
    param_io
    roscpp
    geometry_msgs
    std_msgs
    sensor_msgs
  DEPENDS
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
