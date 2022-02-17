#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_back_laser_publisher");
  ros::NodeHandle n;

  ros::Rate r(25);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, 0.383, 0.924), tf::Vector3(-0.565, -0.3025, 0.0805)),
        ros::Time::now(),"base_link", "laser_frame_back"));
    r.sleep();
  }
}