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

#include "arac_model/State/State.hpp"
#include "arac_model/Model/AracModel.hpp"

#include "arac_msgs/KulmanState.h"

#include <param_io/get_param.hpp>


namespace estimator {

using namespace param_io;
template<typename KulmanModel_>

class KulmanStateEstimatorBase
{
 public:
  KulmanStateEstimatorBase(KulmanModel_& model)
 : model_(model)
  {

  }
  ;

  virtual ~KulmanStateEstimatorBase()
  {

  }

  virtual void initilize(ros::NodeHandle* nh)
  {
    nodeHandle_ = nh;

    readParameters();

    initilizeSubscribers();
    initilizePublishers();

    std::cout << "state_estimator::init " << std::endl;
  }
  ;

  virtual void create()
  {
  }

  virtual void advance(double dt)
  {
  }

  virtual void execute()
  {
  }

  virtual void readParameters()
  {
    getParam(*nodeHandle_, "subscribers/estimator/topic", kulmanDataSubscriberName_);
    getParam(*nodeHandle_, "subscribers/estimator/queue_size", kulmanDataSubscriberQueueSize_);
  }

 protected:
  virtual void initilizeSubscribers()
  {
    kulmanDataSubscriber_ = nodeHandle_->subscribe(kulmanDataSubscriberName_,
                                                 kulmanDataSubscriberQueueSize_,
                                                 &KulmanStateEstimatorBase::getStateMsg, this);
  }

  virtual void initilizePublishers()
  {
  }

  virtual void getStateMsg( arac_msgs::KulmanState msg){
    kulmanStateMsg_ = msg ;
  }


  std::string nodeName_;
  ros::NodeHandle* nodeHandle_;
  ros::Rate* loop_rate_;

  //kuco::State& state_;
  KulmanModel_ model_;

  // Subscriber
  ros::Subscriber kulmanDataSubscriber_;
  // Subscriber names
  std::string kulmanDataSubscriberName_;
  // Subscriber queue_size
  int kulmanDataSubscriberQueueSize_;
  // Subscriber msgs
  arac_msgs::KulmanState kulmanStateMsg_;
};

}
