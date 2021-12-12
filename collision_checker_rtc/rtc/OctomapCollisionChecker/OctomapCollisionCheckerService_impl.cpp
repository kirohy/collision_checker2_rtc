#include "OctomapCollisionCheckerService_impl.h"
#include "OctomapCollisionChecker.h"

OctomapCollisionCheckerService_impl::OctomapCollisionCheckerService_impl()
{
}

OctomapCollisionCheckerService_impl::~OctomapCollisionCheckerService_impl()
{
}

void OctomapCollisionCheckerService_impl::setComp(OctomapCollisionChecker *i_comp)
{
  comp_ = i_comp;
}

void OctomapCollisionCheckerService_impl::setParams(const collision_checker_rtc::OctomapCollisionCheckerService::OctomapCollisionCheckerParam& i_param)
{
  comp_->setParams(i_param);
};

void OctomapCollisionCheckerService_impl::getParams(collision_checker_rtc::OctomapCollisionCheckerService::OctomapCollisionCheckerParam_out i_param)
{
  i_param = new collision_checker_rtc::OctomapCollisionCheckerService::OctomapCollisionCheckerParam();
  comp_->getParams(*i_param);
};

