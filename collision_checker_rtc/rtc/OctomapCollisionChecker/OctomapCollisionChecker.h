#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H

#include <unordered_map>
#include <vector>
#include <memory>
#include <thread>
#include <utility>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Body>

#include <collision_checker_msgs/idl/Collision.hh>
#include <octomap_msgs_rtmros_bridge/idl/Octomap.hh>

#include <moveit/distance_field/propagation_distance_field.h>
#include <octomap_msgs/Octomap.h>

namespace Vclip{
  class Polyhedron;
}

class OctomapCollisionChecker
  : public RTC::DataFlowComponentBase
{
 public:

  OctomapCollisionChecker(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();

  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void octomapCallback(std::shared_ptr<octomap_msgs::Octomap> octomap, cnoid::Position fieldOrigin);

  class boundingBox {
    public:
    cnoid::Position localPose = cnoid::Position::Identity();
    cnoid::LinkPtr parentLink;
    cnoid::Vector3 dimensions = cnoid::Vector3::Zero();

    bool isInside(const cnoid::Vector3f& p) {
      cnoid::Vector3f plocal = worldPoseinv * p;
      return
        (plocal[0] < dimensions[0]/2) &&
        (plocal[1] < dimensions[1]/2) &&
        (plocal[2] < dimensions[2]/2) &&
        (plocal[0] > -dimensions[0]/2) &&
        (plocal[1] > -dimensions[1]/2) &&
        (plocal[2] > -dimensions[2]/2);
    }
    void setParentLinkPose(){
      if(parentLink){
        worldPoseinv = (parentLink->T() * localPose).inverse().cast<float>();
      }
    }
  private:
    Eigen::Affine3f worldPoseinv;
  };

 protected:

  RTC::TimedDoubleSeq m_q_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  RTC::TimedPoint3D m_basePos_;
  RTC::InPort<RTC::TimedPoint3D> m_basePosIn_;
  RTC::TimedOrientation3D m_baseRpy_;
  RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn_;
  octomap_msgs_rtmros_bridge::TimedOctomapWithPose m_octomap_;
  RTC::InPort<octomap_msgs_rtmros_bridge::TimedOctomapWithPose> m_octomapIn_;

  collision_checker_msgs::TimedCollisionSeq m_collision_;
  RTC::OutPort<collision_checker_msgs::TimedCollisionSeq> m_collisionOut_;

 private:
  cnoid::BodyPtr robot_;

  std::shared_ptr<std::thread> thread_;
  bool thread_done_ = true;

  std::shared_ptr<distance_field::PropagationDistanceField> field_;
  cnoid::Position fieldOrigin_ = cnoid::Position::Identity();
  std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Vector3f> > verticesMap_;
  std::vector<cnoid::LinkPtr> targetLinks_;
  std::vector<boundingBox > ignoreBoundingBox_;
};


extern "C"
{
  void OctomapCollisionCheckerInit(RTC::Manager* manager);
};

#endif
