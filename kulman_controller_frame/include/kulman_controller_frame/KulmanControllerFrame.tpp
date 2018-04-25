// arac gazebo
#include "kulman_controller_frame/KulmanControllerFrame.hpp"

namespace kuco {

// Note : param_io is needed to use the getParam
using namespace param_io;

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::KulmanControllerFrame():
    loop_rate_(0)
{
}

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::~KulmanControllerFrame()
{
}

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
void KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::execute()
{
  while (ros::ok()) {
    advance(dt_);
    ros::spinOnce();
    loop_rate_->sleep();
  }
}

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
void KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::advance(double dt)
{
  // Estimator here in future
  estimatorHandler_->advance(dt_);

  // Advance the joystick handler
  joystickHandler_->advance(dt_);

  // Advance the controller
  controller_->advance(dt_);

  // set actuator commands
  setActuatorCommand();

  // publish actuators
  actuatorCommandPublisher_.publish(actuatorCommand_);

}

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
void KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::readParameters()
{
  // Gudumcunun adim suresi okundu
  getParam(*nodeHandle_, "time_step",
           dt_);
  loopFrequency_ = 1/dt_;

  getParam(*nodeHandle_, "robot_name", robotName_);

  // Publisher degiskenleri okundu
  getParam(*nodeHandle_, "publishers/actuator_commands/topic", actuatorCommandPublisherName_);
  getParam(*nodeHandle_, "publishers/actuator_commands/queue_size",
           actuatorCommandPublisherQueueSize_);

  getParam(*nodeHandle_, "joints/jointNumber", n_);
  getParam(*nodeHandle_, "joints/jointNames", jointNames_);

}

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
void KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::initilizePublishers()
{
  std::string publisherName = "/" + robotName_ + actuatorCommandPublisherName_;
  actuatorCommandPublisher_ = nodeHandle_->advertise<kulman_msgs::ActuatorCommands>(
      publisherName, actuatorCommandPublisherQueueSize_);

}

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
void KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::createActuatorCommand()
{
  actuatorCommand_.inputs.name = jointNames_;
  actuatorCommand_.inputs.position = jointVelocities_;
  actuatorCommand_.inputs.velocity = jointVelocities_;
  actuatorCommand_.inputs.effort = jointEffort_;
}

template<typename KulmanModel_,typename Controller_,typename Estimator_,typename Joystick_>
void KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>::print(){
  std::cout << " Controller Frame is initilized\n"
            << " Loop Rate : " << loopFrequency_
            << std::endl ;

}

} /* namespace kuco*/
