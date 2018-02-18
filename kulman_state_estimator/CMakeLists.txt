cmake_minimum_required(VERSION 2.8.3)
project(kulman_state_estimator)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  param_io
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    param_io
)

#############
## Install ##
#############


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
