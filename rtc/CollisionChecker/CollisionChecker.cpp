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
#include <cnoid/SceneDrawables>
#include <cnoid/EigenUtil>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <choreonoid_vclip/choreonoid_vclip.h>

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
    ""
  };
// </rtc-template>

CollisionChecker::CollisionChecker(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
  m_qIn_("q", m_q_),
  m_basePosIn_("basePos", m_basePos_),
  m_baseRpyIn_("baseRpy", m_baseRpy_),
  m_collisionOut_("collisionOut", m_collision_)
{
}

RTC::ReturnCode_t CollisionChecker::onInitialize()
{
  std::cerr << "[" << this->m_profile.instance_name << "] onInitialize()" << std::endl;

  addInPort("q", this->m_qIn_);
  addInPort("basePos", this->m_basePosIn_);
  addInPort("baseRpy", this->m_baseRpyIn_);
  addOutPort("collisionOut", this->m_collisionOut_);

  // load robot model
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

  // calculate convex hull for all collision mesh
  choreonoid_qhull::convertAllCollisionToConvexHull(this->robot_);

  // generate VClip model
  for(int i=0;i<this->robot_->numLinks();i++){
    this->vclipModelMap_[this->robot_->link(i)] = choreonoid_vclip::convertToVClipModel(this->robot_->link(i)->collisionShape());
  }

  // load collision_pair
  std::string collision_pairProp;
  if(this->getProperties().hasKey("collision_pair")) collision_pairProp = std::string(this->getProperties()["collision_pair"]);
  else collision_pairProp = std::string(this->m_pManager->getConfig()["collision_pair"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] collision_pair: " << collision_pairProp <<std::endl;
  std::istringstream iss(collision_pairProp);
  std::string tmp;
  while (getline(iss, tmp, ' ')) {
    size_t pos = tmp.find_first_of(':');
    std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
    cnoid::LinkPtr link1 = this->robot_->link(name1), link2 = this->robot_->link(name2);
    if ( !link1 ) {
      std::cerr << "[" << this->m_profile.instance_name << "] Could not find robot link " << name1 << std::endl;
      continue;
    }
    if ( !link2 ) {
      std::cerr << "[" << this->m_profile.instance_name << "] Could not find robot link " << name2 << std::endl;
      continue;
    }
    std::cerr << "[" << this->m_profile.instance_name << "] check collisions between " << link1->name() << " and " <<  link2->name() << std::endl;
    std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>();
    pair->link1 = link1;
    pair->link2 = link2;
    this->collisionPairs_.push_back(pair);
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionChecker::onExecute(RTC::UniqueId ec_id)
{
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  if (this->m_qIn_.isNew()) this->m_qIn_.read();
  if (this->m_basePosIn_.isNew()) this->m_basePosIn_.read();
  if (this->m_baseRpyIn_.isNew()) this->m_baseRpyIn_.read();

  if(this->m_q_.data.length() == this->robot_->numJoints()){
    for ( int i = 0; i < this->robot_->numJoints(); i++ ){
      this->robot_->joint(i)->q() = this->m_q_.data[i];
    }
  }
  this->robot_->rootLink()->p()[0] = m_basePos_.data.x;
  this->robot_->rootLink()->p()[1] = m_basePos_.data.y;
  this->robot_->rootLink()->p()[2] = m_basePos_.data.z;
  this->robot_->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy_.data.r, m_baseRpy_.data.p, m_baseRpy_.data.y);
  this->robot_->calcForwardKinematics();

  for(size_t i=0;i<this->collisionPairs_.size(); i++){
    std::shared_ptr<CollisionPair>& pair = this->collisionPairs_[i];
    cnoid::Vector3 localp1, localp2;
    double distance;
    choreonoid_vclip::computeDistance(this->vclipModelMap_[pair->link1],
                                      pair->link1->p(),
                                      pair->link1->R(),
                                      this->vclipModelMap_[pair->link2],
                                      pair->link2->p(),
                                      pair->link2->R(),
                                      distance,
                                      localp1,
                                      localp2
                                      );
    if(distance > 1e-6){
      pair->distance = distance;
      pair->direction21 = (pair->link1->T()*localp1 - pair->link2->T()*localp2).normalized();
      pair->localp1 = localp1;
      pair->localp2 = localp2;
    }else{
      // 干渉時は近傍点が正しくない場合があるので、干渉直前の値を使う
      pair->distance = distance;
    }
  }

  this->m_collision_.tm = this->m_q_.tm;
  this->m_collision_.data.length(this->collisionPairs_.size());
  for(size_t i=0;i<this->collisionPairs_.size();i++){
    this->m_collision_.data[i].link1 = this->collisionPairs_[i]->link1->name().c_str();
    this->m_collision_.data[i].point1.x = this->collisionPairs_[i]->localp1[0];
    this->m_collision_.data[i].point1.y = this->collisionPairs_[i]->localp1[1];
    this->m_collision_.data[i].point1.z = this->collisionPairs_[i]->localp1[2];
    this->m_collision_.data[i].link2 = this->collisionPairs_[i]->link2->name().c_str();
    this->m_collision_.data[i].point2.x = this->collisionPairs_[i]->localp2[0];
    this->m_collision_.data[i].point2.y = this->collisionPairs_[i]->localp2[1];
    this->m_collision_.data[i].point2.z = this->collisionPairs_[i]->localp2[2];
    this->m_collision_.data[i].direction21.x = this->collisionPairs_[i]->direction21[0];
    this->m_collision_.data[i].direction21.y = this->collisionPairs_[i]->direction21[1];
    this->m_collision_.data[i].direction21.z = this->collisionPairs_[i]->direction21[2];
    this->m_collision_.data[i].distance = this->collisionPairs_[i]->distance;
  }
  this->m_collisionOut_.write();

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


