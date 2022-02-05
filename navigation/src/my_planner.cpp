#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>
#include <base_local_planner/trajectory_planner_ros.h>

#include <dynamic_reconfigure/server.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "my_planner");

    ros::NodeHandle nh;

    ros::Rate loop_rate(5);
    
    // TF2 objects
    tf2_ros::Buffer tf_buff(ros::Duration(10));
    tf2_ros::TransformListener tf(tf_buff);

    // Initialize costmaps (global and local)
    costmap_2d::Costmap2DROS l_costmap("local_costmap", tf_buff);
    costmap_2d::Costmap2DROS g_costmap("global_costmap", tf_buff);
   
    // Initialize TrajectoryPlannerROS
    base_local_planner::TrajectoryPlannerROS tp;
    tp.initialize("my_planner", &tf_buff, &l_costmap);

    //Start map
    l_costmap.start();
    g_costmap.start();

    
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PoseStamped global_pose;
    geometry_msgs::PoseStamped local_pose;
    
    while(ros::ok()){

        l_costmap.updateMap();
        g_costmap.updateMap();

        //test = tp.checkTrajectory(0, 0, 20);
        
        if(tp.checkTrajectory(0, 0, 20)){
            ROS_WARN("True");;
        }else{
            ROS_WARN("False");
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    
}