#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <collision_checker_msgs/CollisionArray.h>

namespace collision_array_frame_converter {
  class collision_array_frame_converter
  {
  public:
    collision_array_frame_converter() {
      ros::NodeHandle nh, pnh("~");
      pnh.param("from", from_frame_,std::string(""));
      pnh.param("to", to_frame_,std::string(""));

      collisionSub_ = pnh.subscribe("input", 1, &collision_array_frame_converter::collisionCallback, this);
      collisionPub_ = pnh.advertise<collision_checker_msgs::CollisionArray>("output", 1);
    }

    void collisionCallback(const collision_checker_msgs::CollisionArray::ConstPtr& msg){
      tf::StampedTransform transform;
      try{
        tfListener_.lookupTransform(this->to_frame_, this->from_frame_, ros::Time(0), transform);
      } catch (std::exception& ex) {
        ROS_ERROR_STREAM("failed to lookup transform between " << this->to_frame_ << " and " << this->from_frame_);
        return;
      }
      Eigen::Affine3d T;
      tf::transformTFToEigen(transform,T);

      collision_checker_msgs::CollisionArray outmsg = *msg;
      for(int i=0;i<outmsg.collisions.size();i++){
        if(outmsg.collisions[i].point1.header.frame_id == this->from_frame_){
          Eigen::Vector3d p;
          tf::pointMsgToEigen(outmsg.collisions[i].point1.point,p);
          Eigen::Vector3d outp = T * p;
          tf::pointEigenToMsg(outp, outmsg.collisions[i].point1.point);
          outmsg.collisions[i].point1.header.frame_id = this->to_frame_;
        }
        if(outmsg.collisions[i].point2.header.frame_id == this->from_frame_){
          Eigen::Vector3d p;
          tf::pointMsgToEigen(outmsg.collisions[i].point2.point,p);
          Eigen::Vector3d outp = T * p;
          tf::pointEigenToMsg(outp, outmsg.collisions[i].point2.point);
          outmsg.collisions[i].point2.header.frame_id = this->to_frame_;
        }
        if(outmsg.collisions[i].direction21.header.frame_id == this->from_frame_){
          Eigen::Vector3d v;
          tf::vectorMsgToEigen(outmsg.collisions[i].direction21.vector,v);
          Eigen::Vector3d outv = T.linear() * v;
          tf::vectorEigenToMsg(outv, outmsg.collisions[i].direction21.vector);
          outmsg.collisions[i].direction21.header.frame_id = this->to_frame_;
        }
      }
      this->collisionPub_.publish(outmsg);
    }

  protected:
    tf::TransformListener tfListener_;

    ros::Subscriber collisionSub_;
    ros::Publisher collisionPub_;

    std::string from_frame_;
    std::string to_frame_;
  private:
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_array_frame_converter");
  ros::NodeHandle nh;
  collision_array_frame_converter::collision_array_frame_converter c;
  ros::spin();
}
