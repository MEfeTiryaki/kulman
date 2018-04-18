/*
 * joystickHandlerBase.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: efe
 */
#pragma once

#include "kulman_joystick/JoystickHandlerBase.hpp"


namespace joystick {
template<typename KulmanModel_>
class JoystickAcc : public JoystickHandlerBase<KulmanModel_>{
public:
  JoystickAcc(KulmanModel_& model);

  virtual ~JoystickAcc();

  virtual void advance(double dt) override;

protected:

  virtual void getJoystickMsg(geometry_msgs::Twist msg);

 private:

 kuco::Acceleration acceleration_;
 kuco::Acceleration accelerationFree_;

 kuco::AngularAcceleration angularAcceleration_;
 kuco::AngularAcceleration angularAccelerationFree_;

 kuco::Velocity velocity_;
 kuco::Velocity velocityMax_;

 kuco::AngularVelocity angularVelocity_;
 kuco::AngularVelocity angularVelocityMax_;

 double inputSamplingTime_;
};

}

#include "kulman_joystick/JoystickAcc.tpp"
