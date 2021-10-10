#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H

#include <unordered_map>
#include <vector>
#include <memory>
#include <utility>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Body>

#include <collision_checker_rtc/idl/Collision.hh>

namespace Vclip{
  class Polyhedron;
}

class CollisionChecker
  : public RTC::DataFlowComponentBase
{
 public:

  CollisionChecker(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();

  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  class CollisionPair {
  public:
    cnoid::LinkPtr link1;
    cnoid::LinkPtr link2;
    double distance = 0;
    cnoid::Vector3 direction21 = cnoid::Vector3::UnitX();
    cnoid::Vector3 localp1 = cnoid::Vector3::Zero();
    cnoid::Vector3 localp2 = cnoid::Vector3::Zero();
  };

 protected:

  RTC::TimedDoubleSeq m_q_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  RTC::TimedPoint3D m_basePos_;
  RTC::InPort<RTC::TimedPoint3D> m_basePosIn_;
  RTC::TimedOrientation3D m_baseRpy_;
  RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn_;

  collision_checker_rtc::TimedCollisionSeq m_collision_;
  RTC::OutPort<collision_checker_rtc::TimedCollisionSeq> m_collisionOut_;

 private:
  cnoid::BodyPtr robot_;
  std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> > vclipModelMap_;
  std::vector<std::shared_ptr<CollisionPair> > collisionPairs_;
};


extern "C"
{
  void CollisionCheckerInit(RTC::Manager* manager);
};

#endif
