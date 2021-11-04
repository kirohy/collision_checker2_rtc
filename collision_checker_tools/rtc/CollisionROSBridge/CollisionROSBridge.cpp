#include "CollisionROSBridge.h"
#include <tf2/utils.h>

#include <cnoid/BodyLoader>

CollisionROSBridge::CollisionROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_collisionROSOut_("collisionOut", m_collisionROS_),
  m_collisionRTMIn_("collisionIn", m_collisionRTM_)
{
}

RTC::ReturnCode_t CollisionROSBridge::onInitialize(){
  addOutPort("collisionOut", m_collisionROSOut_);
  addInPort("collisionIn", m_collisionRTMIn_);

  cnoid::BodyLoader bodyLoader;

  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  this->robot_vrml_ = bodyLoader.load(fileName);
  if(!this->robot_vrml_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->robot_urdf_ = std::make_shared<urdf::Model>();
  this->robot_urdf_->initParam("robot_description");

  ros::NodeHandle pnh("~");

  if(pnh.hasParam("tf_prefix")){
    pnh.getParam("tf_prefix", this->tf_prefix_);
    if(this->tf_prefix_.size() != 0) this->tf_prefix_ = "/" + this->tf_prefix_ + "/";
  }

  sub_ = pnh.subscribe("input", 1, &CollisionROSBridge::topicCallback, this);
  pub_ = pnh.advertise<collision_checker_msgs::CollisionArray>("output", 1);

  return RTC::RTC_OK;
}

std::string URDFToVRMLLinkName(cnoid::BodyPtr robot_vrml, std::shared_ptr<urdf::Model> robot_urdf, const std::string& URDFLinkName, const std::string& odom){
  std::shared_ptr<const urdf::Link> link = robot_urdf->getLink(URDFLinkName);
  if(link){
    if(link->parent_joint){
      return link->parent_joint->name;
    }else if (link == robot_urdf->getRoot()){
      return robot_vrml->rootLink()->name();
    }
  }else if (URDFLinkName == odom){
    return "";
  }
  std::cerr << "\x1b[31m" << "failed to find link [" << URDFLinkName << "]" << "\x1b[39m" << std::endl;
  return URDFLinkName;
};

std::string VRMLToURDFLinkName(cnoid::BodyPtr robot_vrml, std::shared_ptr<urdf::Model> robot_urdf, const std::string& VRMLLinkName, const std::string& odom){
  std::shared_ptr<const urdf::Joint> joint = robot_urdf->getJoint(VRMLLinkName);
  if(joint){
    return joint->child_link_name;
  }else if (robot_vrml->rootLink()->name() == VRMLLinkName){
    return robot_urdf->getRoot()->name;
  }else if (VRMLLinkName == ""){
    return odom;
  }
  std::cerr << "\x1b[31m" << "failed to find link [" << VRMLLinkName << "]" << "\x1b[39m" << std::endl;
  return VRMLLinkName;
};

RTC::ReturnCode_t CollisionROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_collisionRTMIn_.isNew()){
    this->m_collisionRTMIn_.read();

    collision_checker_msgs::CollisionArray msg;
    msg.header.stamp = ros::Time::now();
    for(int i=0;i<m_collisionRTM_.data.length();i++){
      collision_checker_msgs::Collision state;
      state.point1.header.frame_id = this->tf_prefix_+VRMLToURDFLinkName(this->robot_vrml_, this->robot_urdf_, std::string(m_collisionRTM_.data[i].link1), this->tf_prefix_+"odom");
      state.point1.point.x = m_collisionRTM_.data[i].point1.x;
      state.point1.point.y = m_collisionRTM_.data[i].point1.y;
      state.point1.point.z = m_collisionRTM_.data[i].point1.z;
      state.point2.header.frame_id = this->tf_prefix_+VRMLToURDFLinkName(this->robot_vrml_, this->robot_urdf_, std::string(m_collisionRTM_.data[i].link2), this->tf_prefix_+"odom");
      state.point2.point.x = m_collisionRTM_.data[i].point2.x;
      state.point2.point.y = m_collisionRTM_.data[i].point2.y;
      state.point2.point.z = m_collisionRTM_.data[i].point2.z;
      state.direction21.header.frame_id = this->tf_prefix_+"odom";
      state.direction21.vector.x = m_collisionRTM_.data[i].direction21.x;
      state.direction21.vector.y = m_collisionRTM_.data[i].direction21.y;
      state.direction21.vector.z = m_collisionRTM_.data[i].direction21.z;
      state.distance = m_collisionRTM_.data[i].distance;
      msg.collisions.push_back(state);
    }
    this->pub_.publish(msg);
  }

  return RTC::RTC_OK;
}

void CollisionROSBridge::topicCallback(const collision_checker_msgs::CollisionArray::ConstPtr& msg) {
  coil::TimeValue coiltm(coil::gettimeofday());
  m_collisionROS_.tm.sec  = coiltm.sec();
  m_collisionROS_.tm.nsec = coiltm.usec() * 1000;
  m_collisionROS_.data.length(msg->collisions.size());
  for(int i=0;i<msg->collisions.size();i++){
    m_collisionROS_.data[i].link1 = URDFToVRMLLinkName(this->robot_vrml_, this->robot_urdf_, msg->collisions[i].point1.header.frame_id,this->tf_prefix_+"odom").c_str();
    m_collisionROS_.data[i].point1.x = msg->collisions[i].point1.point.x;
    m_collisionROS_.data[i].point1.y = msg->collisions[i].point1.point.y;
    m_collisionROS_.data[i].point1.z = msg->collisions[i].point1.point.z;
    m_collisionROS_.data[i].link2 = URDFToVRMLLinkName(this->robot_vrml_, this->robot_urdf_, msg->collisions[i].point2.header.frame_id,this->tf_prefix_+"odom").c_str();
    m_collisionROS_.data[i].point2.x = msg->collisions[i].point2.point.x;
    m_collisionROS_.data[i].point2.y = msg->collisions[i].point2.point.y;
    m_collisionROS_.data[i].point2.z = msg->collisions[i].point2.point.z;
    m_collisionROS_.data[i].direction21.x = msg->collisions[i].direction21.vector.x;
    m_collisionROS_.data[i].direction21.y = msg->collisions[i].direction21.vector.y;
    m_collisionROS_.data[i].direction21.z = msg->collisions[i].direction21.vector.z;
    m_collisionROS_.data[i].distance = msg->collisions[i].distance;
  }

  m_collisionROSOut_.write();
}

static const char* CollisionROSBridge_spec[] = {
  "implementation_id", "CollisionROSBridge",
  "type_name",         "CollisionROSBridge",
  "description",       "CollisionROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void CollisionROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(CollisionROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<CollisionROSBridge>, RTC::Delete<CollisionROSBridge>);
    }
};
