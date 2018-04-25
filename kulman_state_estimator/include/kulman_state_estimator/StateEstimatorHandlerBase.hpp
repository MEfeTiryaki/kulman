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

#include "kulman_msgs/KulmanState.h"

#include <param_io/get_param.hpp>

namespace estimator {

using namespace param_io;

template<typename KulmanModel_>

class StateEstimatorHandlerBase
{
 public:
  StateEstimatorHandlerBase(KulmanModel_& model)
      : model_(model)
  {

  }

  virtual ~StateEstimatorHandlerBase()
  {

  }

  virtual void initilize(ros::NodeHandle* nh)
  {
    nodeHandle_ = nh;

    readParameters();

    initilizeSubscribers();
    initilizePublishers();

  }

  virtual void create()
  {
  }

  virtual void advance(double dt)
  {
  }

  virtual void readParameters()
  {

    getParam(*nodeHandle_, "state_estimator_used", isEstimatorUsed_);

    getParam(*nodeHandle_, "robot_name", robotName_);
    std::string kulmanStateSubscriberNameBuffer;
    if (isEstimatorUsed_){
      getParam(*nodeHandle_, "subscribers/estimator/topic", kulmanStateSubscriberNameBuffer);
      getParam(*nodeHandle_, "subscribers/estimator/queue_size", kulmanStateSubscriberQueueSize_);
    } else {
      getParam(*nodeHandle_, "subscribers/kulman_state/topic", kulmanStateSubscriberNameBuffer);
      getParam(*nodeHandle_, "subscribers/kulman_state/queue_size", kulmanStateSubscriberQueueSize_);
    }
    kulmanStateSubscriberName_ = "/" + robotName_ + kulmanStateSubscriberNameBuffer ;
    std::cout << "[" << robotName_ << " StateEstimatorHandler::readParameters] " << kulmanStateSubscriberName_ << std::endl;
  }


 protected:
  virtual void initilizeSubscribers()
  {
    kulmanStateSubscriber_ = nodeHandle_->subscribe(kulmanStateSubscriberName_,
                                                    kulmanStateSubscriberQueueSize_,
                                                    &StateEstimatorHandlerBase::getStateMsg, this);
  }

  virtual void initilizePublishers()
  {
  }

  virtual void getStateMsg(kulman_msgs::KulmanState msg)
  {
    kulmanStateMsg_ = msg;
  }

  ros::NodeHandle* nodeHandle_;
  std::string robotName_;

  // Estimator flag
  bool isEstimatorUsed_;

  //kuco::State& state_;
  KulmanModel_ model_;

  // Subscriber
  ros::Subscriber kulmanStateSubscriber_;
  // Subscriber names
  std::string kulmanStateSubscriberName_;
  // Subscriber queue_size
  int kulmanStateSubscriberQueueSize_;
  // Subscriber msgs
  kulman_msgs::KulmanState kulmanStateMsg_;

};

}
