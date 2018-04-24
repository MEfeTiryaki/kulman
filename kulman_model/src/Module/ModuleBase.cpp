/*
 name : ModuleBase.cpp
 Author : Selçuk Ercan , M. Efe Tiryaki

 */

#include "kulman_model/Module/ModuleBase.hpp"

namespace kuco {

// Note : param_io is needed to use the getParam
// Default Consructor Definition
ModuleBase::ModuleBase():
    desiredState_(new State()),
    measuredState_(new State())
{
}

// Default Destructor Definition
ModuleBase::~ModuleBase()
{
}
void ModuleBase::initilize()
{
}

void ModuleBase::create()
{
}

void ModuleBase::advance(double dt)
{
}

void ModuleBase::setName(std::string name ){
  name_= name;
}
std::string ModuleBase::getName(){
  return name_;
}

State& ModuleBase::getDesiredState()
{
  return *desiredState_;
}
State& ModuleBase::getMeasuredState()
{
  return *measuredState_;
}

} /* namespace kuco*/
