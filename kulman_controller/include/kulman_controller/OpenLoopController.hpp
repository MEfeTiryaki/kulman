#pragma once

#include <vector>


namespace kuco {

template<typename KulmanModel_>
class OpenLoopController
{
 public:
  /*
   * Gudumcuyu yaratir.
   */
  OpenLoopController(KulmanModel_& model)
   :model_(model)
  {
  };

  /*
   * Gudumcuyu yok eder.
   */
  virtual ~OpenLoopController(){};

  /*
   * Gudumcuyu hazirlar
   */
  virtual void initilize(){};

  /*
   * Gudumcu katsayilarinin yaml belgelerinden okur
   */
  virtual void readParameters(){};

  /*
   *  Her gudum dongusunde cagirilmak uzere ilerlme yordami
   */

  virtual void advance(double dt){
    getState();
    calculateInput();
    setActuatorCommand();
  };


 protected:

  /*
   * Bu yordam durum gunceller
   */
  virtual void getState(){};

  /*
   * Bu yordam acik dongu gudum dizgelerinde istenilen
   * girdiyi hesaplamaya yarar.
   */
  virtual void calculateInput(){};

  /*
   * Bu yordam devirtec degerlerini atar.
   */
  virtual void setActuatorCommand(){};

  // Gudulecek kulmanin modeli
  KulmanModel_ model_;


};

} /* kuco adligi*/
