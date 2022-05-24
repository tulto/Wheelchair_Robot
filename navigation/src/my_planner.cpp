#include <string>
#include <ros/ros.h>

#include <services_and_messages/MovementCheck.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>
#include <base_local_planner/trajectory_planner_ros.h>


class WCRSeperatePlanner : public base_local_planner::TrajectoryPlannerROS{
    private:
    bool state_x = false; // true = 0 and false = 0.5
    bool state_y = false; 
    double max_vel_x; // max_vel_x of parameter
    double max_vel_y;
    bool initial_locate = 0;
    bool locate_state = 0;
    ros::Time start;
    ros::Time cmd_vel_time;

    ros::ServiceServer service_locate;
    ros::ServiceServer check_movement;
    ros::Publisher pub_vel;
    ros::Subscriber sub_vel;

    geometry_msgs::Twist vel_msg;
    geometry_msgs::Twist vel;

    char* appendCharToCharArray(char* array, char a)
    {
        size_t len = strlen(array);

        char* ret = new char[len+2];

        strcpy(ret, array);    
        ret[len] = a;
        ret[len+1] = '\0';

        return ret;
    }


    public:
    WCRSeperatePlanner(ros::NodeHandle *nh) 
    {
        pub_vel = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sub_vel = nh->subscribe("/cmd_vel", 5, &WCRSeperatePlanner::callback_vel, this);
        service_locate = nh->advertiseService("/locate_service",&WCRSeperatePlanner::locate_service, this);
        check_movement = nh->advertiseService("/check_movement",&WCRSeperatePlanner::check_movement_service, this);
    }

    /**
     * @brief if a 180° rotation is possible set min_vel_x to 0 else -0.5
     * 
     */
    void check_rotation(){
        // check possibility in tuning trajectory 
        bool traj_state = checkTrajectory(0, 0, 10);
        
        // if Trajecotry is possible (true) then use velocity 0 else 0.5
        // setting velocities over cml with dynamic_reconfigure
        if(traj_state == true && state_x == false){
            state_x = true;
            
            ROS_WARN("min_vel_x auf 0");
            system("rosrun dynamic_reconfigure dynparam set move_base/DWAPlannerROS min_vel_x 0");

        }else if(traj_state == false && state_x == true){
            state_x = false;
            ROS_WARN("min_vel_x auf -0.5");
            system("rosrun dynamic_reconfigure dynparam set move_base/DWAPlannerROS min_vel_x -0.5");
        }
    }

    /**
     * @brief Check if vehicle can rotate drive forward and backward then no lateral movement necessary
     * 
     */
    void allow_lateral_if_necessary(){
        bool traj_state;
        // check possibility in tuning trajectory 
        if (checkTrajectory(0.3, 0, 0) && checkTrajectory(-0.3, 0, 0) && checkTrajectory(0, 0, 10)){
            traj_state = true;
        }else{
            traj_state = false;
        }
        
        // if Trajecotry is possible (true) then use velocity 0 else 0.5
        // setting velocities over cml with dynamic_reconfigure
        if(traj_state == true && state_y == false){
            state_y = true;
            ROS_WARN("min_vel_y auf 0");
            system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS \"{\'max_vel_y\':\'0\', \'min_vel_y\':\'0\'}\"");


        }else if(traj_state == false && state_y == true){
            state_y = false;
            ROS_WARN("min_vel_y auf 0.3");
            system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS \"{\'max_vel_y\':\'0.25\', \'min_vel_y\':\'-0.25\'}\"");
        }
    }

    /**
     * @brief A possibility to let the vehicle selflocate in form from driving itself throgh the environment
     * 
     * @param duration 
     */
    void locate(int duration = 30){ 
        bool check_tra = 0;

        vel_msg.linear.x = 0.15;
        vel_msg.linear.y = 0;
        vel_msg.angular.z = 0.35;
        if (locate_state == 1){
            check_tra = checkTrajectory(vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
            if (check_tra){
                pub_vel.publish(vel_msg);
            }
            else{
                vel_msg.linear.x = -vel_msg.linear.x;
                vel_msg.linear.y = vel_msg.linear.y;
                vel_msg.angular.z = vel_msg.angular.z;
                check_tra = checkTrajectory(vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
                if (check_tra){
                    pub_vel.publish(vel_msg);
                }else{
                    get_out_of_everywhere();
                }
            }
        }

        if (locate_state == 1 && (ros::Time::now().sec - start.sec) > duration){
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.angular.z = 0;
            pub_vel.publish(vel_msg);
            system("rosrun dynamic_reconfigure dynparam set /my_planner/local_costmap/static_layer enabled True");
            locate_state = 0;
        }
    }

    /**
     * @brief Get the out of everywhere object  Score multiple velocities and only use best performing velociies
     * 
     */
    void get_out_of_everywhere(){
        double vel[3];
        double score_max=0;
        
        for (double x = -0.3; x <= 0.3; x = x+0.1){
            for (double y = -0.2; y<=0.2; y = y+0.1){
                for(double t=-0.4; t<=0.4; t= t+0.1){
                    double score = scoreTrajectory(x,y,t); 
                    if (score_max < score){
                        score_max = score; 
                        vel[0] = x;
                        vel[1] = y;
                        vel[2] = t;
                    }
                }
            }
        }

        vel_msg.linear.x = vel[0];
        vel_msg.linear.y = vel[1];
        vel_msg.angular.z = vel[2];
        if (checkTrajectory(vel[0], vel[1], vel[2])){
            pub_vel.publish(vel_msg);
        }else{
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.angular.z = 0;
            pub_vel.publish(vel_msg);
        }
    }

    /**
     * @brief calback funktion of cmd_vel
     * 
     * @param msg
     */
    void callback_vel(const geometry_msgs::Twist::ConstPtr& msg){
        cmd_vel_time = ros::Time::now();
    }

    /**
     * @brief get cmd_vel to 0 if nothing is sent for a predefinded time
     * 
     * @param time  time in Seconds
     */
    void check_cmd_vel(double time = 0.5){
        if (ros::Time::now().toSec() - cmd_vel_time.toSec() > time){
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.angular.z = 0;
            pub_vel.publish(vel_msg);
        }
    }

    /**
     * @brief activate self localization
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool locate_service(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res){ 
        initial_locate = 1;
        locate_state = 1;
        start = ros::Time::now();
        return true;
    }

    /**
     * @brief a service for checking velocities (if valid it returns true)
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool check_movement_service(services_and_messages::MovementCheckRequest &req,
                        services_and_messages::MovementCheckResponse &res){                    
        if (checkTrajectory(req.x, req.y, req.z)){
            res.check = true;
        }else{
            res.check = false;
        }
        return true;
    }
};



int main(int argc, char **argv){

    ros::init(argc, argv, "my_planner");

    ros::NodeHandle nh;

    ros::Rate loop_rate(10);
    
    // TF2 objects
    tf2_ros::Buffer tf_buff(ros::Duration(2));
    tf2_ros::TransformListener tf(tf_buff);

    // Initialize costmaps
    costmap_2d::Costmap2DROS l_costmap("local_costmap", tf_buff);
   
    // Initialize TrajectoryPlannerROS
    WCRSeperatePlanner tp(&nh);
    tp.initialize("my_planner", &tf_buff, &l_costmap);

    //Start map
    l_costmap.start();

    bool state = false; // true = 0 and false = 0.5

    double max_vel_x; // max_vel_x of parameter
    
    while(ros::ok()){

        l_costmap.updateMap(); // update local_costmap

        tp.check_rotation();
        tp.allow_lateral_if_necessary();
        tp.check_cmd_vel(0.5);
        tp.locate(30);

        loop_rate.sleep();
        ros::spinOnce();
    }

    
}