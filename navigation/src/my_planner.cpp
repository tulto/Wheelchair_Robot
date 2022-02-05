#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>
#include <base_local_planner/trajectory_planner_ros.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "my_planner");

    ros::NodeHandle nh;

    ros::Rate loop_rate(5);
    
    // TF2 objects
    tf2_ros::Buffer tf_buff(ros::Duration(2));
    tf2_ros::TransformListener tf(tf_buff);

    // Initialize costmaps
    costmap_2d::Costmap2DROS l_costmap("local_costmap", tf_buff);
   
    // Initialize TrajectoryPlannerROS
    base_local_planner::TrajectoryPlannerROS tp;
    tp.initialize("my_planner", &tf_buff, &l_costmap);

    //Start map
    l_costmap.start();

    bool state = false; // true = 0 and false = 0.5

    double max_vel_x; // max_vel_x of parameter
    
    while(ros::ok()){

        l_costmap.updateMap(); // update local_costmap

        // check possibility in tuning trajectory 
        bool traj_state = tp.checkTrajectory(0, 0, 20);
        
        // if Trajecotry is possible (true) then use velocity 0 else 0.5
        // setting velocities over cml with dynamic_reconfigure
        if(traj_state == true && state == false){
            state = true;
            ROS_WARN("min_vel_x auf 0");
            system("rosrun dynamic_reconfigure dynparam set move_base/DWAPlannerROS min_vel_x 0");

        }else if(traj_state == false && state == true){
            state = false;
            ROS_WARN("min_vel_x auf -0.5");
            system("rosrun dynamic_reconfigure dynparam set move_base/DWAPlannerROS min_vel_x -0.5");
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    
}