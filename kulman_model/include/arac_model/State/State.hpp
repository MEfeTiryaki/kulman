/*
 name : State.hpp
 Author : Selçuk Ercan , M. Efe Tiryaki

 */

#include <Eigen/Core>

#include <iostream>
#include <vector>

#pragma once

namespace kuco {
// Todo (Efe Tiryaki 16.02.18): Bu arkadaşlaro typedef headerı içinde başka bir yerde tanımla
// daha genel kullanıma açılsın
typedef Eigen::Vector3d Position;
typedef Eigen::Vector4d Quaternion;
typedef Eigen::Vector3d Velocity;
typedef Eigen::Vector3d AngularVelocity;
typedef Eigen::Vector3d Acceleration;
typedef Eigen::Vector3d AngularAcceleration;
typedef Eigen::Vector3d Force;
typedef Eigen::Vector3d Torque;

class State
{
 public:
  State();

  virtual ~State();

  const Position& getPositionInWorldFrame() const;

  void setPositionInWorldFrame(const Position& pos);

  //
  const Quaternion& getOrientationInWorldFrame() const;

  void setOrientationInWorldFrame(const Quaternion& q);

  //
  const Velocity& getVelocityInWorldFrame() const;

  void setVelocityInWorldFrame(const Velocity& vel);

  //
  const AngularVelocity& getAngularVelocityInWorldFrame() const;

  void setAngularVelocityInWorldFrame(const AngularVelocity& angVel);

  //
  const Acceleration& getAccelerationInWorldFrame() const ;
  void setAccelerationInWorldFrame(const Acceleration& acc);

  const AngularAcceleration& getAngularAccelerationInWorldFrame()const;
  void setAngularAccelerationInWorldFrame( const AngularAcceleration& angAcc);

  const Force& getForceInWorldFrame()const;
  void setForceInWorldFrame(const Force& f);

  const Torque& getTorqueInWorldFrame()const;
  void setTorqueInWorldFrame(const Torque& t);

 protected:

  Position positionInWorldFrame_;
  Quaternion orientationInWorldFrame_;
  Velocity velocityInWorldFrame_;
  AngularVelocity angularVelocityInWorldFrame_;
  Acceleration accelerationInWorldFrame_;
  AngularAcceleration angularAccelerationInWorldFrame_;
  Force forceInWorldFrame_;
  Torque torqueInWorldFrame_;

};

}/* namespace kuco*/
