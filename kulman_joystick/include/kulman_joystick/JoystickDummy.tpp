#include "kulman_joystick/JoystickDummy.hpp"

namespace joystick {

template<typename KulmanModel_>
JoystickDummy<KulmanModel_>::JoystickDummy(kuco::AracModel& model)
    : JoystickHandlerBase<KulmanModel_>(model)
{
}


template<typename KulmanModel_>
JoystickDummy<KulmanModel_>::~JoystickDummy()
{
}

template<typename KulmanModel_>
void JoystickDummy<KulmanModel_>::advance(double dt)
{
  kuco::Velocity velocity = kuco::Velocity::Zero();
  kuco::AngularVelocity angularVelocity = kuco::AngularVelocity::Zero();

  if (ros::Time::now().toSec() - joystickCommandStartTime_ > 0.5) {
    //velocity = {0.0,0.0,0.0};
    //angularVelocity = {0.0,0.0,0.0};
  } else {
    velocity << joystickMsg_.linear.x, 0.0 , 0.0 ;
    angularVelocity << 0.0 , 0.0 , joystickMsg_.angular.z;
  }

  model_.getBody().getDesiredState().setVelocityInWorldFrame(velocity);
  model_.getBody().getDesiredState().setAngularVelocityInWorldFrame(angularVelocity);

}

}
