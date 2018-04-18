/*
 name : ModuleBase.hpp
 Author : Selçuk Ercan , M. Efe Tiryaki

*/


#pragma once

#include <string>
#include <vector>
#include "arac_model/State/State.hpp"

/* Kulman Controller namespace */
namespace kuco {


class ModuleBase
{
 public:
  // Constructor.
  ModuleBase();

  // Destructor.
  virtual ~ModuleBase();

  // Init
  virtual void initilize();

  // Create
  virtual void create();

  // Advance
  virtual void advance(double dt);

  // Bu işlevler çocukarda değişmeyeceği için virtual tanımlanmadı.
  std::string getName();
  void setName(std::string name);

  State& getDesiredState() ;
  State& getMeasuredState() ;

 private:

  std::string name_;

  State* measuredState_;
  State* desiredState_;

};

} /* namespace kuco */
