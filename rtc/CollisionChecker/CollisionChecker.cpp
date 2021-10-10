// -*- C++ -*-
/*!
 * @file  CollisionChecker.cpp
 * @brief soft error limiter
 * $Date$
 *
 * $Id$
 */

#include "CollisionChecker.h"
#include <rtm/CorbaNaming.h>

#include <cnoid/BodyLoader>

// Module specification
// <rtc-template block="module_spec">
static const char* CollisionChecker_spec[] =
  {
    "implementation_id", "CollisionChecker",
    "type_name",         "CollisionChecker",
    "description",       "CollisionChecker",
    "version",           "0.0.0",
    "vendor",            "Naoki-Hiraoka",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

CollisionChecker::CollisionChecker(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qIn_("q", m_q_)
{
}

RTC::ReturnCode_t CollisionChecker::onInitialize()
{
  std::cerr << "[" << this->m_profile.instance_name << "] onInitialize()" << std::endl;

  addInPort("q", m_qIn_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  // load collision_pair
  std::string collision_pairProp;
  if(this->getProperties().hasKey("collision_pair")) collision_pairProp = std::string(this->getProperties()["collision_pair"]);
  else collision_pairProp = std::string(this->m_pManager->getConfig()["collision_pair"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] collision_pair: " << collision_pairProp <<std::endl;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionChecker::onExecute(RTC::UniqueId ec_id)
{
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  if (m_qIn_.isNew()) {
    m_qIn_.read();
  }

  return RTC::RTC_OK;
}

extern "C"
{

  void CollisionCheckerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(CollisionChecker_spec);
    manager->registerFactory(profile,
                             RTC::Create<CollisionChecker>,
                             RTC::Delete<CollisionChecker>);
  }

};


