#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


int main (int argc, char** argv) {
  ros::init(argc, argv, "robot_dimensions");
  ros::NodeHandle n;
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 100 );
  ros::Rate r(20);

  while (ros::ok())
  {

      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0.129;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1.15;
      marker.scale.y = 0.74;
      marker.scale.z = 0.258;
      marker.color.a = 0.5; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      vis_pub.publish( marker );

  }
}