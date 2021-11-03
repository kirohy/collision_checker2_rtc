#include <ros/ros.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

class octomap_collision_check {
public:
  octomap_collision_check(){
    ros::NodeHandle nh, pnh("~");
    octomapSub_ = nh.subscribe("binary_octomap", 1, &octomap_collision_check::octomapCallback, this);
    markerPub_ = pnh.advertise<visualization_msgs::Marker>("markers",1);
  }

protected:
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

  ros::Subscriber octomapSub_;
  ros::Publisher markerPub_;

  std::shared_ptr<distance_field::PropagationDistanceField> field_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_collision_check");
  octomap_collision_check c;

  ros::spin();

  // distance_field::PropagationDistanceField field(10.0,10.0,10.0,
  //                                                0.02,
  //                                                0.0,0.0,0.0,
  //                                                1.0,
  //                                                false);

  return 0;
}
