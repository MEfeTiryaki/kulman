#pragma once

#include <ros/ros.h>

// ROS messages / services
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <vector>

#include <param_io/get_param.hpp>


class ActuatorContainer
{
 public:
  // Constructor.
  ActuatorContainer();

  // Destructor.
  virtual ~ActuatorContainer();



};
