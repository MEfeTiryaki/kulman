/*
 name : Body.hpp
 Author : Selçuk Ercan , M. Efe Tiryaki

*/

#pragma once

#include "arac_model/Module/ModuleBase.hpp"

/* Kulman Controller namespace */
namespace kuco {


class Body : public ModuleBase
{
 public:
  // Constructor.
  Body();

  // Destructor.
  virtual ~Body();

  // Init
  virtual void initilize();

  // Create
  virtual void create();

  // Advance
  virtual void advance(double dt);



};

} /* namespace kuco */
