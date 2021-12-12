// -*-C++-*-
#ifndef OctomapCollisionCheckerSERVICESVC_IMPL_H
#define OctomapCollisionCheckerSERVICESVC_IMPL_H

#include "collision_checker_rtc/idl/OctomapCollisionCheckerService.hh"

class OctomapCollisionChecker;

class OctomapCollisionCheckerService_impl
  : public virtual POA_collision_checker_rtc::OctomapCollisionCheckerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  OctomapCollisionCheckerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~OctomapCollisionCheckerService_impl();
  void setParams(const collision_checker_rtc::OctomapCollisionCheckerService::OctomapCollisionCheckerParam& i_param);
  void getParams(collision_checker_rtc::OctomapCollisionCheckerService::OctomapCollisionCheckerParam_out i_param);
  //
  void setComp(OctomapCollisionChecker *i_comp);
private:
  OctomapCollisionChecker *comp_;
};

#endif
