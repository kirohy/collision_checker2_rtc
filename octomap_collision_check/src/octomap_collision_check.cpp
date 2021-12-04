#include <ros/ros.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <urdf/model.h>
#include <cnoid/BodyLoader>
#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <map>
#include <vector>
#include <unordered_set>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <collision_checker_msgs/CollisionArray.h>
#include <std_msgs/String.h>

class octomap_collision_check {
public:
  octomap_collision_check(){
    ros::NodeHandle nh, pnh("~");
    octomapSub_ = pnh.subscribe("octomap", 1, &octomap_collision_check::octomapCallback, this);
    linkNameSub_ = pnh.subscribe("linknames", 1, &octomap_collision_check::linkNamesCallback, this);
    markerPub_ = pnh.advertise<visualization_msgs::Marker>("markers",1);
    collisionPub_ = pnh.advertise<collision_checker_msgs::CollisionArray>("collisions",1);

    robot_urdf_.initParam("robot_description");
    std::string fileName;
    pnh.getParam("model", fileName);
    cnoid::BodyLoader bodyLoader;
    robot_vrml_ = bodyLoader.load(fileName);
    if(!this->robot_vrml_){
      ROS_FATAL_STREAM("failed to load model[" << fileName << "]");
      exit(1);
    }

    float resolution = 0.01;
    for(int i=0;i<robot_vrml_->numLinks();i++){
      cnoid::LinkPtr link = robot_vrml_->link(i);
      std::vector<cnoid::Vector3f> vertices; // 同じvertexが2回カウントされている TODO
      cnoid::SgMeshPtr mesh = octomap_collision_check::convertToSgMesh(link->collisionShape());
      if(mesh) {
        for(int j=0;j<mesh->numTriangles();j++){
          cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
          cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
          cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
          float l1 = (v1 - v0).norm();
          float l2 = (v2 - v0).norm();
          cnoid::Vector3f n1 = (v1 - v0).normalized();
          cnoid::Vector3f n2 = (v2 - v0).normalized();
          for(int m=0;m<l1;m++){
            for(int n=0;n<l2-l2/l1*m;n++){
              vertices.push_back(v0 + n1 * m + n2 * n);
            }
            int n=l2-l2/l1*m;
            vertices.push_back(v0 + n1 * m + n2 * n);
          }
          int m = l1;
          int n= 0;
          vertices.push_back(v0 + n1 * m + n2 * n);
        }
      }

      verticesMap_[link] = vertices;
    }

    linkNames_ = std::make_shared<std::vector<std::string> >();
    for(int i=0;i<robot_vrml_->numLinks();i++){
      cnoid::LinkPtr link = robot_vrml_->link(i);
      if(verticesMap_[link].size() == 0) continue;
      std::shared_ptr<const urdf::Link> urdflink = octomap_collision_check::getURDFLinkFromVRMLName(robot_vrml_,robot_urdf_,link->name());
      if(!urdflink) continue;
      linkNames_->push_back(urdflink->name);
    }

    periodicCollisionCheckTimer_ = nh.createTimer(ros::Duration(1.0 / 25),
                                                  boost::bind(&octomap_collision_check::periodicCollisionCheckTimerCallback, this, _1));
  }

protected:
  void linkNamesCallback(const std_msgs::String::ConstPtr& msg) {
    std::shared_ptr<std::vector<std::string> > linkNames = std::make_shared<std::vector<std::string> >();
    std::stringstream ss(msg->data);
    std::string item;
    while (std::getline(ss, item, ',')) {
      cnoid::LinkPtr link = octomap_collision_check::getVRMLLinkFromURDFName(this->robot_vrml_,this->robot_urdf_,item);
      if(!link) continue;
      if(this->verticesMap_[link].size() == 0) continue;
      linkNames->push_back(item);
    }
    this->linkNames_ = linkNames;
  }

  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));

    if(octree){
      this->fieldFrameId_ = msg->header.frame_id;
      double minx,miny,minz; octree->getMetricMin(minx,miny,minz);
      double maxx,maxy,maxz; octree->getMetricMax(maxx,maxy,maxz);
      this->field_ = std::make_shared<distance_field::PropagationDistanceField>(*octree, octomap::point3d(minx,miny,minz),octomap::point3d(maxx,maxy,maxz), 1.0, false);
    }else{
      this->field_ = nullptr;
    }

    if(this->field_ && this->markerPub_.getNumSubscribers() > 0){
      visualization_msgs::Marker marker;
      this->field_->getIsoSurfaceMarkers(0.1, 0.12, msg->header.frame_id, msg->header.stamp,marker);
      this->markerPub_.publish(marker);
    }
  }

  void periodicCollisionCheckTimerCallback(const ros::TimerEvent& event){
    // octomapCallbackが別スレッドで上書きしてもいいようにコピー
    std::shared_ptr<distance_field::PropagationDistanceField> field = this->field_;
    std::string fieldFrameId = this->fieldFrameId_;
    if(!field) return;

    collision_checker_msgs::CollisionArray msg;
    msg.header.frame_id = fieldFrameId;
    msg.header.stamp = ros::Time::now();

    std::shared_ptr<std::vector<std::string> > linkNames = this->linkNames_;
    for(int i=0;i<linkNames->size();i++){
      tf::StampedTransform transform;
      try{
        tfListener_.lookupTransform(fieldFrameId, (*linkNames)[i], ros::Time(0), transform);
      } catch (std::exception& ex) {
        continue;
      }
      cnoid::LinkPtr link = octomap_collision_check::getVRMLLinkFromURDFName(this->robot_vrml_,this->robot_urdf_,(*linkNames)[i]);
      if(!link) continue;
      Eigen::Affine3d linkT_d;
      tf::transformTFToEigen(transform,linkT_d);
      Eigen::Affine3f linkT = linkT_d.cast<float>();

      double min_dist = 1.0;
      cnoid::Vector3f closest_v = cnoid::Vector3f::Zero();
      cnoid::Vector3 closest_point = cnoid::Vector3::Zero();
      cnoid::Vector3 closest_direction = cnoid::Vector3::UnitX();

      const std::vector<cnoid::Vector3f>& vertices = this->verticesMap_[link];
      for(int j=0;j<vertices.size();j++){
        cnoid::Vector3f v = linkT * vertices[j];
        cnoid::Vector3 grad;
        bool in_bound;
        double dist = this->field_->getDistanceGradient(v[0],v[1],v[2],grad[0],grad[1],grad[2],in_bound);
        if(in_bound && grad.norm() > 0){
          if(dist < min_dist){
            closest_direction[0] = (grad[0]/grad.norm());
            closest_direction[1] = (grad[1]/grad.norm());
            closest_direction[2] = (grad[2]/grad.norm());
            closest_point[0] = v[0]-closest_direction[0]*dist;
            closest_point[1] = v[1]-closest_direction[1]*dist;
            closest_point[2] = v[2]-closest_direction[2]*dist;
            min_dist = dist;
            closest_v = vertices[j];
          }
        }
      }

      if(min_dist < 1.0 && min_dist > 0.0){
        collision_checker_msgs::Collision collision;
        collision.point1.header.frame_id = (*linkNames)[i];
        collision.point1.point.x = closest_v[0];
        collision.point1.point.y = closest_v[1];
        collision.point1.point.z = closest_v[2];
        collision.point2.header.frame_id = fieldFrameId;
        collision.point2.point.x = closest_point[0];
        collision.point2.point.y = closest_point[1];
        collision.point2.point.z = closest_point[2];
        collision.direction21.header.frame_id = fieldFrameId;
        collision.direction21.vector.x = closest_direction[0];
        collision.direction21.vector.y = closest_direction[1];
        collision.direction21.vector.z = closest_direction[2];
        collision.distance = min_dist;
        msg.collisions.push_back(collision);
      }
    }
    collisionPub_.publish(msg);
  }

  static void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->getOrCreateVertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }

  static cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){
    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
      model->setName(collisionshape->name());
    }else{
      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }

    return model;
  }

  static std::shared_ptr<const urdf::Link> getURDFLinkFromVRMLName(cnoid::BodyPtr robot_vrml, urdf::Model& robot_urdf, const std::string& VRMLLinkName){
    std::shared_ptr<const urdf::Joint> joint = robot_urdf.getJoint(VRMLLinkName);
    if(joint){
      return robot_urdf.getLink(joint->child_link_name);
    }else if (robot_vrml->rootLink()->name() == VRMLLinkName){
      return robot_urdf.getRoot();
    }else{
      return nullptr;
    }
  }
  static cnoid::LinkPtr getVRMLLinkFromURDFName(cnoid::BodyPtr robot_vrml, urdf::Model& robot_urdf, const std::string& URDFLinkName){
    std::shared_ptr<const urdf::Link> link = robot_urdf.getLink(URDFLinkName);
    if(link){
      if(link->parent_joint){
        return robot_vrml->link(link->parent_joint->name);
      }else if (link == robot_urdf.getRoot()){
        return robot_vrml->rootLink();
      }
    }
    return nullptr;
  }

  ros::Timer periodicCollisionCheckTimer_;
  ros::Subscriber octomapSub_;
  ros::Subscriber linkNameSub_;
  ros::Publisher markerPub_;
  ros::Publisher collisionPub_;
  tf::TransformListener tfListener_;

  std::shared_ptr<distance_field::PropagationDistanceField> field_;
  std::string fieldFrameId_;
  urdf::Model robot_urdf_;
  cnoid::BodyPtr robot_vrml_;
  std::map<cnoid::LinkPtr, std::vector<cnoid::Vector3f> > verticesMap_;
  std::shared_ptr<std::vector<std::string> > linkNames_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_collision_check");
  octomap_collision_check c;

  // octomapのsubscribe処理には時間がかかるため、片方のspinnerがoctomap用、もう片方がcollision check用として別スレッドで処理することを意図
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
