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
    getParam(*nodeHandle_, "subscribers/estimator/topic"     , kulmanDataSubscriberName_);
    getParam(*nodeHandle_, "subscribers/estimator/queue_size", kulmanDataSubscriberQueueSize_);

    getParam(*nodeHandle_, "publishers/kulman_state/topic"     , kulmanStateEstimatorPublisherName_);
    getParam(*nodeHandle_, "publishers/kulman_state/queue_size", kulmanStateEstimatorPublisherQueueSize_);
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
    kulmanStateEstimatorPublisher_ = nodeHandle_->advertise<kulman_msgs::KulmanState>(
        kulmanStateEstimatorPublisherName_, kulmanStateEstimatorPublisherQueueSize_);
  }

  virtual void getStateMsg( kulman_msgs::KulmanState msg){
    kulmanStateMsg_ = msg ;
  }

  virtual void publishEstimatedState()
  {
    kulmanStateEstimatorPublisher_.publish(kulmanStateEstimatorMsg_);
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
  kulman_msgs::KulmanState kulmanStateMsg_;

  // Publisher
  ros::Publisher kulmanStateEstimatorPublisher_;
  // Publisher names
  std::string kulmanStateEstimatorPublisherName_;
  // Publisher queue_size
  int kulmanStateEstimatorPublisherQueueSize_;
  // Publisher msgs
  kulman_msgs::KulmanState kulmanStateEstimatorMsg_ ;

};

}
