#include "kulman_joystick/JoystickAcc.hpp"

namespace joystick {

template<typename KulmanModel_>
JoystickAcc<KulmanModel_>::JoystickAcc(KulmanModel_& model)
    : JoystickHandlerBase<KulmanModel_>(model),
      acceleration_(),
      accelerationFree_(kuco::Acceleration::Ones() * 0.005),
      angularAcceleration_(),
      angularAccelerationFree_(kuco::AngularAcceleration::Ones() * 0.002),
      velocity_(),
      velocityMax_(kuco::Velocity::Ones() * 10),
      angularVelocity_(),
      angularVelocityMax_(kuco::AngularVelocity::Ones() * 5),
      inputSamplingTime_(0.01)
{
}

template<typename KulmanModel_>
JoystickAcc<KulmanModel_>::~JoystickAcc()
{
}

template<typename KulmanModel_>
void JoystickAcc<KulmanModel_>::advance(double dt)
{
  const double acc = 10;
  const double angularAcc = 10;
  const double accFree = 3;
  const double angularAccFree = 3;

  bool isAccFree = false;
  bool isAngularAccFree = false;

  double dtSample_ = inputSamplingTime_;

  // Todo : change the name of the  joystickCommandStartTime_
  if (ros::Time::now().toSec() - this->joystickCommandStartTime_ >= inputSamplingTime_) {
    if (this->joystickMsg_.linear.x > 0) {
      acceleration_[0] = acc;
    } else if (this->joystickMsg_.linear.x < 0) {
      acceleration_[0] = -acc;
    } else {
      isAccFree = true;
      if (velocity_[0] > 0) {
        acceleration_[0] = -accFree;
      } else if (velocity_[0] < 0) {
        acceleration_[0] = accFree;
      } else {
        acceleration_[0] = 0.0;
      }
    }
    if (this->joystickMsg_.angular.z > 0) {
      angularAcceleration_[2] = angularAcc;
    } else if (this->joystickMsg_.angular.z < 0) {
      angularAcceleration_[2] = -angularAcc;
    } else {
      isAngularAccFree = true;
      if (angularVelocity_[2] > 0) {
        angularAcceleration_[2] = -angularAccFree;
      } else if (angularVelocity_[2] < 0) {
        angularAcceleration_[2] = angularAccFree;
      } else {
        angularAcceleration_[2] = 0.0;
      }
    }
    // Reset the Velocity Commands Back to 0
    this->joystickMsg_.linear.x = 0.0;
    this->joystickMsg_.angular.z = 0.0;

    if (!isAccFree) {
      if ((velocity_[0] + acceleration_[0] * dtSample_ < velocityMax_[0])
          && (velocity_[0] + acceleration_[0] * dtSample_ > -velocityMax_[0])) {
        velocity_[0] += acceleration_[0] * dtSample_;
      } else if (velocity_[0] + acceleration_[0] * dtSample_ >= velocityMax_[0]) {
        velocity_[0] = velocityMax_[0];
      } else {
        velocity_[0] = -velocityMax_[0];
      }
    } else {

      if ((velocity_[0] + acceleration_[0] * dtSample_) * velocity_[0] > 0.0) {
        if ((velocity_[0] + acceleration_[0] * dtSample_ < velocityMax_[0])
            && (velocity_[0] + acceleration_[0] * dtSample_ > -velocityMax_[0])) {
          velocity_[0] += acceleration_[0] * dtSample_;
        } else if (velocity_[0] + acceleration_[0] * dtSample_ >= velocityMax_[0]) {
          velocity_[0] = velocityMax_[0];
        } else {
          velocity_[0] = -velocityMax_[0];
        }
      } else {
        velocity_[0] = 0.0;
      }
      isAccFree = false;
    }
    if (!isAngularAccFree) {
      if ((angularVelocity_[2] + angularAcceleration_[2] * dtSample_ < angularVelocityMax_[2])
          && (angularVelocity_[2] + angularAcceleration_[2] * dtSample_ > -angularVelocityMax_[2])) {
        angularVelocity_[2] += angularAcceleration_[2] * dtSample_;
      } else if (angularVelocity_[0] + angularAcceleration_[2] * dtSample_ >= angularVelocityMax_[2]) {
        angularVelocity_[2] = angularVelocityMax_[2];
      } else {
        angularVelocity_[2] = -angularVelocityMax_[2];
      }
    } else {
      if ((angularVelocity_[2] + angularAcceleration_[2] * dtSample_) * angularVelocity_[2] > 0.0) {
        if ((angularVelocity_[2] + angularAcceleration_[2] * dtSample_ < angularVelocityMax_[2])
            && (angularVelocity_[2] + angularAcceleration_[2] * dtSample_ > -angularVelocityMax_[2])) {
          angularVelocity_[2] += angularAcceleration_[2] * dtSample_;
        } else if (angularVelocity_[2] + angularAcceleration_[2] * dtSample_ >= angularVelocityMax_[2]) {
          angularVelocity_[2] = angularVelocityMax_[2];
        } else {
          angularVelocity_[2] = -angularVelocityMax_[2];
        }
      } else {
        angularVelocity_[2] = 0.0;
      }
      isAngularAccFree = false;
    }

    this->joystickCommandStartTime_ = ros::Time::now().toSec();
  }
  // ELSE DO NOTHING

  /*
  std::cout << "__________________________________________________________________________"
      << std::endl;
  std::cout << "time : " << ros::Time::now().toSec() << std::endl;
  std::cout << "acceleration : " << acceleration_.transpose() << std::endl;
  std::cout << "angularAcceleration : " << angularAcceleration_.transpose() << std::endl;
  std::cout << "velocity : " << velocity_.transpose() << std::endl;
  std::cout << "angularVelociy : " << angularVelocity_.transpose() << std::endl;
  //*/

  this->model_.getBody().getDesiredState().setVelocityInWorldFrame(velocity_);
  this->model_.getBody().getDesiredState().setAngularVelocityInWorldFrame(angularVelocity_);

}

template<typename KulmanModel_>
void JoystickAcc<KulmanModel_>::getJoystickMsg(geometry_msgs::Twist msg)
{
  this->joystickMsg_ = msg;
  //joystickCommandStartTime_ = ros::Time::now().toSec();
}

}
