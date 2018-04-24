/*
 name : Kulman.hpp
 Author : Selçuk Ercan , M. Efe Tiryaki

*/

#include "kulman_model/Module/Body.hpp"
#pragma once


namespace kuco {

class KulmanModel
{
 public:
   KulmanModel();

   virtual ~KulmanModel();

   virtual void initilize();

   virtual void create();

   virtual void advance(double dt);

   virtual void reset();

  void setBody(Body* Body);
  Body& getBody();

 protected:

   // Her kulmanın gövdesi olması gerektigi için gövdeyi kulmanda tanımlıyoruz
   // bu class'tan çıkan çocuklar da kullana bilsin diye protected olarak tanımlandı
   Body* Body_;

};

} /* namespace kuco */
