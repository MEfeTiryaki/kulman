/*
 name : Wheel.hpp
 Author : Selçuk Ercan , M. Efe Tiryaki

*/

#pragma once

#include "arac_model/Module/ModuleBase.hpp"

/* Kulman Controller namespace */
namespace kuco {


class Wheel: public ModuleBase
{
 public:
  // Constructor.
  Wheel();

  // Destructor.
  virtual ~Wheel();

  // Init
  virtual void initilize();

  // Create
  virtual void create();

  // Advance
  virtual void advance(double dt);




};

} /* namespace kuco */
