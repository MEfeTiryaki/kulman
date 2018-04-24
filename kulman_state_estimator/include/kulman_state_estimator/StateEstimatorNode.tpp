// arac gazebo
#include "kulman_state_estimator/StateEstimatorNode.hpp"

namespace estimator {

// Note : param_io is needed to use the getParam
using namespace param_io;

template<typename KulmanModel_,typename Estimator_>
StateEstimatorNode<KulmanModel_,Estimator_>::StateEstimatorNode():
    loop_rate_(0)
{
}

template<typename KulmanModel_,typename Estimator_>
StateEstimatorNode<KulmanModel_,Estimator_>::~StateEstimatorNode()
{
}

template<typename KulmanModel_,typename Estimator_>
void StateEstimatorNode<KulmanModel_,Estimator_>::create()
{
  model_ = new KulmanModel_();
  estimator_ = new Estimator_(*model_);
}

template<typename KulmanModel_,typename Estimator_>
void StateEstimatorNode<KulmanModel_,Estimator_>::initilize(int argc, char **argv)
{
  // nodeHandler olusturuldu.
  nodeName_ = "/kulman_state_estimator";
  ros::init(argc, argv, nodeName_);
  nodeHandle_ = new ros::NodeHandle("~");
  nodeName_ = ros::this_node::getName();

  // Parametreler okundu.
  readParameters();

  // loop rate ayarlandi
  loop_rate_ = new ros::Rate(loopFrequency_);

  initilizePublishers();
  initilizeSubscribers();

  model_->initilize();
  estimator_->initilize(nodeHandle_);

  print();
}

template<typename KulmanModel_,typename Estimator_>
void StateEstimatorNode<KulmanModel_,Estimator_>::execute()
{
  while (ros::ok()) {
    advance(dt_);
    ros::spinOnce();
    loop_rate_->sleep();
  }
}

template<typename KulmanModel_,typename Estimator_>
void StateEstimatorNode<KulmanModel_,Estimator_>::advance(double dt)
{
  // Estimator here in future
  estimator_->advance(dt_);
}

template<typename KulmanModel_,typename Estimator_>
void StateEstimatorNode<KulmanModel_,Estimator_>::readParameters()
{
  // Gudumcunun adim suresi okundu
  getParam(*nodeHandle_, "time_step",
           dt_);
  loopFrequency_ = 1/dt_;

  getParam(*nodeHandle_, "robot_name", robotName_);
  std::string kulmanStatePublisherNameBuffer;
  // Publisher degiskenleri okundu
  getParam(*nodeHandle_, "publishers/kulman_state/topic", kulmanStatePublisherNameBuffer);
  getParam(*nodeHandle_, "publishers/kulman_state/queue_size",
           kulmanStatePublisherQueueSize_);
  kulmanStatePublisherName_ = "/" + robotName_ + kulmanStatePublisherNameBuffer ;

}

template<typename KulmanModel_,typename Estimator_>
void StateEstimatorNode<KulmanModel_,Estimator_>::initilizePublishers()
{
  kulmanStatePublisher_ = nodeHandle_->advertise<kulman_msgs::KulmanState>(
      kulmanStatePublisherName_, kulmanStatePublisherQueueSize_);

}


template<typename KulmanModel_,typename Estimator_>
void StateEstimatorNode<KulmanModel_,Estimator_>::print(){
  std::cout << " State Estimator is initilized\n"
            << " Loop Rate : " << loopFrequency_
            << std::endl ;

}

} /* namespace estimator*/
