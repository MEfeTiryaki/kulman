#pragma once

#include <ros/ros.h>

// ROS messages / services
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include <param_io/get_param.hpp>

#include "kulman_msgs/ActuatorCommands.h"

// stl
#include <memory>

namespace kuco{

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
class KulmanControllerFrame
{
 public:
  // Constructor.
  KulmanControllerFrame();

  // Destructor.
  virtual ~KulmanControllerFrame();

  // Init
  virtual void initilize(int argc, char **argv){};

  // Create
  virtual void create(){};

  // Update
  virtual void update(){};

  // Parameters init
  virtual void readParameters();

  // excute
  virtual void execute();

  // excute
  virtual void advance(double dt);

 protected:

  virtual void initilizePublishers();

  virtual void initilizeSubscribers(){};

  virtual void setActuatorCommand(){};

  void createActuatorCommand();

  // Debug amacli basma yordami
  void print();

protected:

  ros::NodeHandle* nodeHandle_;

  ros::Rate* loop_rate_;
  double dt_;
  double loopFrequency_;

  std::string nodeName_;
  std::string robotName_;

  Estimator_* estimator_;
  Joystick_* joystickHandler_ ;
  Controller_* controller_ ;
  KulmanModel_* model_;


  // Publisher
  ros::Publisher actuatorCommandPublisher_;
  // Publisher names
  std::string actuatorCommandPublisherName_;
  // Publisher queue_size
  int actuatorCommandPublisherQueueSize_;
  // Publisher msgs
  kulman_msgs::ActuatorCommands actuatorCommand_;

  double joystickCommandStartTime_;


  // Joint number
  int n_ ;

  std::vector<std::string> jointNames_;
  std::vector<double> jointPositions_;
  std::vector<double> jointVelocities_;
  std::vector<double> jointEffort_;


};

}

#include "kulman_controller_frame/KulmanControllerFrame.tpp"
