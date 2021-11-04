#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <moveit_msgs/PlanningSceneWorld.h>

class octomap_to_planningsceneworld {
public:
  octomap_to_planningsceneworld(){
    ros::NodeHandle nh, pnh("~");
    octomapSub_ = pnh.subscribe("octomap", 1, &octomap_to_planningsceneworld::octomapCallback, this); //binary
    planningsceneworldPub_ = pnh.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world",1);
  }

protected:
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    moveit_msgs::PlanningSceneWorld outmsg;
    outmsg.octomap.header = msg->header;
    outmsg.octomap.origin.position.x = 0.0;
    outmsg.octomap.origin.position.y = 0.0;
    outmsg.octomap.origin.position.z = 0.0;
    outmsg.octomap.origin.orientation.x = 0.0;
    outmsg.octomap.origin.orientation.y = 0.0;
    outmsg.octomap.origin.orientation.z = 0.0;
    outmsg.octomap.origin.orientation.w = 1.0;
    outmsg.octomap.octomap = *msg;
    this->planningsceneworldPub_.publish(outmsg);
  }

  ros::Subscriber octomapSub_;
  ros::Publisher planningsceneworldPub_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_to_planningsceneworld");
  octomap_to_planningsceneworld c;

  ros::spin();

  return 0;
}
