/*
 name : KulmanModel.cpp
 Author : Selçuk Ercan , M. Efe Tiryaki

*/


#include "kulman_model/Model/KulmanModel.hpp"

namespace kuco {

// Note : param_io is needed to use the getParam

KulmanModel::KulmanModel():
    Body_(new Body())
{
}

KulmanModel::~KulmanModel()
{
}

void KulmanModel::initilize()
{
}

void KulmanModel::create()
{
}

void KulmanModel::advance(double dt)
{
}

void KulmanModel::reset()
{
}

void KulmanModel::setBody(Body* Body)
{
  Body_ = Body;
}

Body& KulmanModel::getBody()
{
  return *Body_;
}

}; /* namespace arac_controller_frame*/
