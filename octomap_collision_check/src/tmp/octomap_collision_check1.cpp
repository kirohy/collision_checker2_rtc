#include <ros/ros.h>
#include <moveit/collision_distance_field/collision_detector_allocator_distance_field.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/transform_listener.h>

class octomap_collision_check {
public:
  octomap_collision_check(){
    ros::NodeHandle nh, pnh("~");

    this->tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
    this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer_);
    this->planningSceneMonitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("/robot_description",this->tfBuffer_);
    this->planningSceneMonitor_->startStateMonitor("/joint_states","attached_collision_object");
    this->planningSceneMonitor_->startWorldGeometryMonitor("collision_object","planning_scene_world");
    this->planningSceneMonitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,"monitored_planning_scene");

    planning_scene_monitor::LockedPlanningSceneRW ls(planningSceneMonitor_);
    ls->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorDistanceField::create(), true);

    markerPub_ = pnh.advertise<visualization_msgs::MarkerArray>("markers",1);
    markerPub1_ = pnh.advertise<visualization_msgs::MarkerArray>("markers1",1);

    periodicCollisionCheckTimer_ = nh.createTimer(ros::Duration(1.0 / 25),
                                                  boost::bind(&octomap_collision_check::periodicCollisionCheckTimerCallback, this, _1));
  }

protected:
  void periodicCollisionCheckTimerCallback(const ros::TimerEvent& event){
    planning_scene_monitor::LockedPlanningSceneRO ls(planningSceneMonitor_);
    const collision_detection::CollisionWorldDistanceFieldConstPtr world = std::dynamic_pointer_cast<const collision_detection::CollisionWorldDistanceField>(ls->getCollisionWorld());

    collision_detection::GroupStateRepresentationPtr gsr;
    collision_detection::CollisionRequest req;
    req.group_name = "fullbody";
    req.distance = true;
    req.contacts = true;
    req.verbose = true;
    collision_detection::CollisionResult res;

    world->getCollisionGradients(req,res,
                                 *(ls->getCollisionRobot()),
                                 ls->getCurrentState(),
                                 nullptr,
                                 gsr);

    visualization_msgs::MarkerArray marker;
    collision_detection::getProximityGradientMarkers("odom", "", ros::Duration(1),
                                                     gsr->link_body_decompositions_, gsr->attached_body_decompositions_,
                                                     gsr->gradients_, marker);
    markerPub_.publish(marker);
    collision_detection::getCollisionMarkers("odom", "", ros::Duration(1),
                                             gsr->link_body_decompositions_, gsr->attached_body_decompositions_,
                                             gsr->gradients_, marker);
    markerPub1_.publish(marker);
  }

  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));

    if(octree){
      double minx,miny,minz; octree->getMetricMin(minx,miny,minz);
      double maxx,maxy,maxz; octree->getMetricMax(maxx,maxy,maxz);
      this->field_ = std::make_shared<distance_field::PropagationDistanceField>(*octree, octomap::point3d(minx,miny,minz),octomap::point3d(maxx,maxy,maxz), 1.0, false);
    }else{
      this->field_ = nullptr;
    }

    if(this->field_){
      {
        visualization_msgs::Marker marker;
        this->field_->getIsoSurfaceMarkers(0.1, 0.12, msg->header.frame_id, msg->header.stamp,marker);
        this->markerPub_.publish(marker);
      }
    }
  }

  ros::Timer periodicCollisionCheckTimer_;
  ros::Publisher markerPub_;
  ros::Publisher markerPub1_;

  std::shared_ptr<distance_field::PropagationDistanceField> field_;

  planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_collision_check");
  octomap_collision_check c;

  ros::spin();

  return 0;
}
