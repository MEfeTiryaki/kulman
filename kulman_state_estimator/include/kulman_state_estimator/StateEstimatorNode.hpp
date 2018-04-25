#pragma once

#include <ros/ros.h>

// ROS messages / services

#include <param_io/get_param.hpp>

#include "kulman_msgs/KulmanState.h"

namespace estimator {

template<typename KulmanModel_,typename Estimator_>
class StateEstimatorNode
{
 public:
  // Constructor.
  StateEstimatorNode();

  // Destructor.
  virtual ~StateEstimatorNode();

  // Init
  virtual void initilize(int argc, char **argv);

  // Create
  virtual void create();

  // Update
  virtual void update()
  {
  }
  ;

  // Parameters init
  virtual void readParameters();

  // excute
  virtual void execute();

  // excute
  virtual void advance(double dt);

 protected:

  virtual void initilizePublishers();

  virtual void initilizeSubscribers()
  {
  }
  ;

  // Debug amacli basma yordami
  void print();

 protected:

  ros::NodeHandle* nodeHandle_;
  std::string nodeName_;
  std::string robotName_;

  ros::Rate* loop_rate_;
  double dt_;
  double loopFrequency_;

  KulmanModel_* model_;
  Estimator_* estimator_;

  // Publisher
  ros::Publisher kulmanStatePublisher_;
  // Publisher names
  std::string kulmanStatePublisherName_;
  // Publisher queue_size
  int kulmanStatePublisherQueueSize_;
  // Publisher msgs
  kulman_msgs::KulmanState actuatorCommand_;

  double joystickCommandStartTime_;

};

}

#include "kulman_state_estimator/StateEstimatorNode.tpp"
