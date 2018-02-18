#include <ros/ros.h>

// ROS messages / services
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <memory>
#include <mutex>

#include "arac_model/State/State.hpp"
#include "arac_model/Model/AracModel.hpp"

namespace estimator {

template<typename KulmanModel>
class KulmanStateEstimatorBase
{
 public:
  KulmanStateEstimatorBase()
  {

  }
  ;

  virtual ~KulmanStateEstimatorBase()
  {

  }

  virtual void initilize(int argc, char **argv)
  {
    nodeName_ = "/state_estimator";

    ros::init(argc, argv, nodeName_);

    nodeHandle_ = new ros::NodeHandle("~");

    // Todo (Efe Tiryaki 18.02.18) : rate'i değiştirilebilir yap
    loop_rate_ = new ros::Rate(400);

    readParameters();
    initilizePublishers();
    initilizeSubscribers();

    std::cout << "state_estimator::init " << std::endl;
  };

  virtual void create(){
    ;
  }

  virtual void advance(){
    ;
  }

  virtual void execute(){
    ;
  }

  virtual void readParameters(){
    ;
  }

 protected:
  virtual void initilizeSubscribers(){
    ;
  }

  virtual void initilizePublishers(){
    ;
  }

  std::string nodeName_;
  ros::NodeHandle* nodeHandle_;
  ros::Rate* loop_rate_;

  //kuco::State& state_;
  KulmanModel model_;
};

}
