/*
 name : State.cpp
 Author : Selçuk Ercan , M. Efe Tiryaki

 */

#include "arac_model/State/State.hpp"

namespace kuco {

State::State()
    : positionInWorldFrame_(),
      orientationInWorldFrame_(),
      velocityInWorldFrame_(),
      angularVelocityInWorldFrame_(),
      accelerationInWorldFrame_(),
      angularAccelerationInWorldFrame_(),
      forceInWorldFrame_(),
      torqueInWorldFrame_()
{
}

State::~State()
{
}

//
const Position&  State::getPositionInWorldFrame() const
{
  return positionInWorldFrame_;
}

void State::setPositionInWorldFrame( const Position& pos)
{
  positionInWorldFrame_ = pos;
}

//
const Quaternion& State::getOrientationInWorldFrame() const
{
  return orientationInWorldFrame_;
}

void State::setOrientationInWorldFrame(const Quaternion& q)
{
  orientationInWorldFrame_ = q;
}

//
const Velocity& State::getVelocityInWorldFrame() const
{
  return velocityInWorldFrame_;
}

void State::setVelocityInWorldFrame(const Velocity& vel)
{
  velocityInWorldFrame_= vel;
}

//
const AngularVelocity& State::getAngularVelocityInWorldFrame() const
{
  return angularVelocityInWorldFrame_;
}

void State::setAngularVelocityInWorldFrame(const AngularVelocity& angVel)
{
  angularVelocityInWorldFrame_ = angVel;
}

//
const Acceleration& State::getAccelerationInWorldFrame() const
{
  return accelerationInWorldFrame_;
}
void State::setAccelerationInWorldFrame(const Acceleration&  acc)
{
  accelerationInWorldFrame_ = acc;
}

const AngularAcceleration&  State::getAngularAccelerationInWorldFrame() const
{
  return angularAccelerationInWorldFrame_;
}
void State::setAngularAccelerationInWorldFrame(const AngularAcceleration&  angAcc)
{
  angularAccelerationInWorldFrame_ = angAcc;
}

const Force& State::getForceInWorldFrame() const
{
  return forceInWorldFrame_;
}
void State::setForceInWorldFrame(const Force&  f)
{
  forceInWorldFrame_ = f;
}

const Torque&  State::getTorqueInWorldFrame() const
{
  return torqueInWorldFrame_;
}
void State::setTorqueInWorldFrame(const Torque& t)
{
  torqueInWorldFrame_ = t;
}

//std::vector<double> State::getVelocityInBaseFrame()
//{
//  return velocityInBaseFrame_;
//}
//void State::setVelocityInBaseFrame(std::vector<double> vel)
//{
//  velocityInBaseFrame_ = vel;
//}
//
//std::vector<double> State::getAngularVelocityInBaseFrame()
//{
//  return angularVelocityInBaseFrame_;
//}
//void State::setAngularVelocityIBaseFrame(std::vector<double> angVel)
//{
//  angularVelocityInBaseFrame_ = angVel;
//}
//
//std::vector<double> State::getAccelerationInBaseFrame()
//{
//  return accelerationInBaseFrame_;
//}
//void State::setAccelerationInBaseFrame(std::vector<double> acc)
//{
//  accelerationInBaseFrame_ = acc;
//}
//
//std::vector<double> State::getAngularAccelerationInBaseFrame()
//{
//  return angularAccelerationInBaseFrame_;
//}
//void State::setAngularAccelerationInBaserame(std::vector<double> angAcc)
//{
//  angularAccelerationInBaseFrame_ = angAcc;
//}
//
//std::vector<double> State::getForceInBaseFrame()
//{
//  return forceInBaseFrame_;
//}
//void State::setForceInBaseFrame(std::vector<double> f)
//{
//  forceInBaseFrame_ = f;
//}
//
//std::vector<double> State::getTorqueInBaseFrame()
//{
//  return torqueInBaseFrame_;
//}
//void State::setTorqueInBaserame(std::vector<double> t)
//{
//  torqueInBaseFrame_ = t;
//}

}
;
/* namespace kuco*/
