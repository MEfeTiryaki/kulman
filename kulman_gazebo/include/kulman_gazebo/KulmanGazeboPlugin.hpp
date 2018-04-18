/*
 Author : M. Efe Tiryaki
 */

#pragma once

// c++
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <condition_variable>
#include <vector>

// required for std::this_thread
#include <thread>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/Element.hh>
// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
// geometry msgs
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// sensor msgs
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "arac_msgs/KulmanState.h"
#include "arac_msgs/ActuatorCommands.h"

#include <tf/transform_broadcaster.h>

// urdf
#include <urdf/model.h>

namespace gazebo {
class KulmanGazeboPlugin : public ModelPlugin
{
 public:
  // Constructor.
  KulmanGazeboPlugin();

  // Destructor.
  virtual ~KulmanGazeboPlugin();

  // Implements Gazebo virtual load function.
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/);

  // Overrides Gazebo init function.
  virtual void Init();

  // Overrides Gazebo reset function.
  virtual void Reset();

  virtual void OnUpdate();

 protected:
  // Reads parameters from the parameter server.
  virtual void readParameters(sdf::ElementPtr sdf);

  virtual void initJointStructures() ;

  virtual void initLinkStructure() ;

  // Inits the ROS subscriber.
  virtual void initSubscribers() ;

  // Inits the ROS publishers.
  virtual void initPublishers() ;

  // Inits the ROS services.
  virtual void initServices(){};

  // Reads simulation state.
  virtual void readSimulation();

  // Writes simulation state.
  virtual void writeSimulation() = 0;

  // TF publish
  virtual void publishTFs() ;

  virtual void publishPublishers();

  // Retrieves URDF robot description from ROS parameter server.
  virtual std::string getUrdfRobotDescription(const std::string& paramName) const;

  // Set Commands to be writen in writeSimulation
  void actuatorCommandsCallback(const arac_msgs::ActuatorCommands& msg);


  // Ros node
  ros::NodeHandle* nodeHandle_;

  // Ensures gazebo methods are called sequentially
  std::recursive_mutex gazeboMutex_;

  // Name of the robot.
  std::string robotName_;
  // ROS robot description parameter name.
  std::string robotDescriptionParamName_;
  // Robot base link.
  std::string robotBaseLink_;
  // ROS robot description URDF string.
  std::string robotDescriptionUrdfString_;
  // ROS robot description URDF model.
  urdf::Model robotDescriptionUrdfModel_;

  // Frame names.
  std::string frameBase_;
  std::string frameOdometry_;
  std::string frameWorld_;

  // TF transforms
  tf::Transform odomTransform;

  // Publisher
  ros::Publisher kulmanStatePublisher_;
  ros::Publisher jointStatePublisher_;
  // Publisher names
  std::string kulmanStatePublisherName_;
  std::string jointStatePublisherName_;
  // Publisher queue_size
  int kulmanStatePublisherQueueSize_;
  int jointStatePublisherQueueSize_;
  // Publisher msgs
  arac_msgs::KulmanState kulmanStateMsg_ ;
  sensor_msgs::JointState jointStates_ ;

  // Subscriber
  ros::Subscriber actuatorCommandSubscriber_;
  // Subscriber names
  std::string actuatorCommandSubscriberName_;
  // Subscriber queue_size
  int actuatorCommandSubscriberQueueSize_;
  // Subscriber msgs
  arac_msgs::ActuatorCommands actuatorCommands_;


  // Estimator Bool
  bool isEstimatorUsed;

  // Model.
  physics::ModelPtr model_;
  // World update event.
  event::ConnectionPtr updateConnection_;

  // Robot links
  physics::LinkPtr baseLink_;

  // Actuators
  std::vector<std::string> jointNames_;
  std::unordered_map<std::string, int> jointNametoJointId_;

  std::vector<gazebo::physics::JointPtr> jointPtrs_;
  std::vector<int> jointTypes_;
  std::vector<double> jointPositionsReset_;
  std::vector<double> jointPositionsDefault_;


  // Gazebo time step.
  double gazeboTimeStep_ = 0.0;
  // Time step for publishing simulation state.
  double publishingTimeStep_ = 0.0;
  // Simulation time stamp taken at the start of the last updateCb() function call.
  common::Time lastStartUpdateSimTime_;
  // System time stamp taken at the start of the last updateCb() function call.
  std::chrono::time_point<std::chrono::steady_clock> lastStartUpdateSysTime_;
  // Current inter-update simulation time step.
  double updateSimTimeStep_ = 0.0;

};

}
