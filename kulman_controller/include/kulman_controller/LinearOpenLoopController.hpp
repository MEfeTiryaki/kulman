#pragma once

#include "kulman_controller/OpenLoopController.hpp"

namespace kuco {

template<typename KulmanModel_>
class LinearOpenLoopController : public OpenLoopController<KulmanModel_>
{
 public:
  /*
   * Gudumcuyu yaratir.
   */
  LinearOpenLoopController(KulmanModel_& model)
      : OpenLoopController<KulmanModel_>(model),
        n_(0),
        l_(0)
  {
  }
  ;

  /*
   * Gudumcuyu yok eder.
   */
  virtual ~LinearOpenLoopController()
  {
  }
  ;

 protected:

  /*
   * Bu yordam acik dongu gudum dizgelerinde istenilen
   * girdiyi hesaplamaya yarar.
   */
  virtual void calculateInput() override
  {
    u_ = B_.inverse() * x_;
  }
  ;

  // Kulman Durumu
  Eigen::VectorXd x_;

  // Kulman girdiler
  Eigen::VectorXd u_;

  // x = B_ * u
  Eigen::MatrixXd B_;

  // Durum uzayinin eleman sayisi
  int n_;

  // Girdi sayisi
  int l_;

};

} /* kuco adligi*/
