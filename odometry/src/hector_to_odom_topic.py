#!/usr/bin/env python3

#### IMPORTS ####

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist, Vector3

#___________________________________________________________________________________________________

#### CALLBACK-FUNCTION ####

def callback_hector_pose(hec_pose):
        
        global last_timestamp, last_rotat, x_last, y_last, theta_last
        
        if hec_pose.header.stamp > last_timestamp:
                                                
                #calculate euler from quaternion 
                euler_rotat = euler_from_quaternion([hec_pose.pose.pose.orientation.x,
                                                        hec_pose.pose.pose.orientation.y,
                                                        hec_pose.pose.pose.orientation.z,
                                                        hec_pose.pose.pose.orientation.w])[2]
                
                #calculate time difference
                time_diff = (hec_pose.header.stamp - last_timestamp).to_sec()
                #calculate velocitys
                x_vel = ((hec_pose.pose.pose.position.x - x_last)/time_diff)
                y_vel = ((hec_pose.pose.pose.position.y - y_last)/time_diff)
                theta_vel = ((euler_rotat - last_rotat)/time_diff)
                
                #pass hec_pose.pose.position to x_last and y_last for velocity calculation
                x_last = hec_pose.pose.pose.position.x
                y_last = hec_pose.pose.pose.position.y
                #pass rotat in euler to last_rotat for velocity calculation
                last_rotat = euler_rotat
                #pass hec_pose.header.stamp to last_timestamp
                last_timestamp = hec_pose.header.stamp
                
                #fill the odom_msg for publishing the odom from hector topic
                odom_msg.header.stamp = rospy.Time.now()  #timestamp for the published msg
                odom_msg.pose.pose = hec_pose.pose.pose
                
                #checking if the pose msg should use a static covariance msg or the one passed from hector mapping
                if use_static_pose_covariance:
                        odom_msg.pose.covariance = static_pose_covariance
                else:
                        odom_msg.pose.covariance = hec_pose.pose.covariance
                odom_msg.twist.twist = Twist(Vector3(x_vel, y_vel, 0), Vector3(0, 0, theta_vel))
                odom_msg.twist.covariance = static_twist_covariance
                                
                #publish odom from hector topic 
                odom_pub.publish(odom_msg)
        
        

#___________________________________________________________________________________________________

#### MAIN-CODE ####

if __name__ == '__main__':
        #initiate ROS node
        rospy.init_node("hector_to_odom_topic_publisher")

        #get parameters from rosparam server
        node_name = rospy.get_name()
        parent_frame = rospy.get_param(node_name + "/parent_frame", "odom")
        child_frame = rospy.get_param(node_name + "/child_frame", "base_link")
        odom_topic_name = rospy.get_param(node_name + "/odom_topic_name", "odom/hector")
        
        #decide if you want to use static pose covariance
        use_static_pose_covariance = rospy.get_param(node_name + "/use_static_pose_covariance", False)
        
        #only change the static covariance value for the main axis so
        #you don't have to give a whole covariance matrix with 36 values
        static_pose_covariance_val = rospy.get_param(node_name + "/static_pose_covariance_val", 0.2)
        static_twist_covariance_val = rospy.get_param(node_name + "/static_twist_covariance_val", 0.4)
        
        #if needed you can change the whole static_covariance matrix through rosparam
        #if not needed it defaults to the values seen below
        static_pose_covariance = rospy.get_param(node_name + "/static_pose_covariance", 
                                                [static_pose_covariance_val, 0, 0, 0, 0, 0,
                                                0, static_pose_covariance_val, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, static_pose_covariance_val])
        
        static_twist_covariance = rospy.get_param(node_name + "/static_twist_covariance",
                                                  [static_twist_covariance_val, 0, 0, 0, 0, 0,
                                                   0, static_twist_covariance_val, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, static_twist_covariance_val])
                
        #define an odometry publisher
        odom_pub = rospy.Publisher(odom_topic_name, Odometry, queue_size=25)
        
        #instance of an odometry message
        odom_msg = Odometry()
        odom_msg.header.frame_id = parent_frame
        odom_msg.child_frame_id = child_frame
        
        #definde variables needed for publishing odom topic
        x_vel, y_vel, theta_vel = 0.0, 0.0, 0.0 #velocity
        last_timestamp = rospy.Time.now()       #save last time that we received a tf
        x_last, y_last = 0.0, 0.0               #save last translation
        last_rotat = 0.0                        #save last rotation in euler
                        
        #subscribe to hector poseupdate
        rospy.Subscriber("poseupdate", PoseWithCovarianceStamped, callback_hector_pose, queue_size = 25)
        
        rospy.spin()
                             
#___________________________________________________________________________________________________
      
        