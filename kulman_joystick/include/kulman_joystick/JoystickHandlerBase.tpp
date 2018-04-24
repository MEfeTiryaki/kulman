

#include "kulman_joystick/JoystickHandlerBase.hpp"



#include <param_io/get_param.hpp>


// Note : param_io is needed to use the getParam

namespace joystick{

using namespace param_io;

template<typename KulmanModel_>
JoystickHandlerBase<KulmanModel_>::JoystickHandlerBase(KulmanModel_& model)
  : model_(model)
{
}

template<typename KulmanModel_>
JoystickHandlerBase<KulmanModel_>::~JoystickHandlerBase(){
}

template<typename KulmanModel_>
void JoystickHandlerBase<KulmanModel_>::initilize(ros::NodeHandle* nh )
{

  nodeHandle_ = nh ;

  readParameters();

  initilizeSubscribers();

  joystickCommandStartTime_ = ros::Time::now().toSec();

}

template<typename KulmanModel_>
void JoystickHandlerBase<KulmanModel_>::advance(double dt){
}

template<typename KulmanModel_>
void JoystickHandlerBase<KulmanModel_>::readParameters(){
  // Get Subscriber parameters
  getParam(*nodeHandle_, "subscribers/joystick/topic", joystickSubscriberName_);
  getParam(*nodeHandle_, "subscribers/joystick/queue_size", joystickSubscriberQueueSize_);

}

template<typename KulmanModel_>
void JoystickHandlerBase<KulmanModel_>::initilizeSubscribers()
{
  joystickSubscriber_ = nodeHandle_->subscribe(joystickSubscriberName_, joystickSubscriberQueueSize_,
                                               &JoystickHandlerBase::getJoystickMsg, this);
}

template<typename KulmanModel_>
void JoystickHandlerBase<KulmanModel_>::getJoystickMsg(geometry_msgs::Twist msg){
  joystickMsg_ = msg;
  joystickCommandStartTime_ = ros::Time::now().toSec();
}


}
