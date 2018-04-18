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
class JoystickDummy: public JoystickHandlerBase<KulmanModel_>{
public:
  JoystickDummy(KulmanModel_& model);

  virtual ~JoystickDummy();

  virtual void advance(double dt) override;

 private:


};

}

#include "kulman_joystick/JoystickDummy.tpp"
