#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Body>

class CollisionChecker
  : public RTC::DataFlowComponentBase
{
 public:

  CollisionChecker(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();

  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:

  RTC::TimedDoubleSeq m_q_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;

 private:
  cnoid::BodyPtr robot_;
};


extern "C"
{
  void CollisionCheckerInit(RTC::Manager* manager);
};

#endif
