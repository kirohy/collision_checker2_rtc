#ifndef CollisionROSBridge_H
#define CollisionROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <collision_checker_msgs/idl/Collision.hh>

#include <ros/ros.h>
#include <collision_checker_msgs/CollisionArray.h>

#include <urdf/model.h>
#include <cnoid/Body>

class CollisionROSBridge : public RTC::DataFlowComponentBase{
protected:
  std::shared_ptr<urdf::Model> robot_urdf_;
  cnoid::BodyPtr robot_vrml_;

  collision_checker_msgs::TimedCollisionSeq m_collisionRTM_;
  RTC::InPort <collision_checker_msgs::TimedCollisionSeq> m_collisionRTMIn_;
  ros::Publisher pub_;

  ros::Subscriber sub_;
  collision_checker_msgs::TimedCollisionSeq m_collisionROS_;
  RTC::OutPort <collision_checker_msgs::TimedCollisionSeq> m_collisionROSOut_;
public:
  CollisionROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void topicCallback(const collision_checker_msgs::CollisionArray::ConstPtr& msg);
};


extern "C"
{
  void CollisionROSBridgeInit(RTC::Manager* manager);
};

#endif // CollisionROSBridge_H
