import rospy
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID

def callback_is_autonomous_nav_active(msg):
	msg.status_list[0].status
	if(last_status == 1 and msg.status_list[0].status == 0):
		
		stop_msg.stamp.nsecs = 0
		stop_msg.stamp.secs = 0
		stop_msg.id = ""

		for i in range(10):
			stop_pub.publish(stop_msg)
			rospy.sleep(0.05)
	
	last_status = msg.status_list[0].status #give status to last_status


if __name__ == '__main__':
	rospy.init_node('reset_cmd_vel_node', anonymous=False)

	last_status = 0;

	sub_mb = rospy.Subscriber("/move_base/status", GoalStatusArray, callback_is_autonomous_nav_active)
	stop_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=5)

	stop_msg = GoalID()	#create a GoalID msg object for publishing the stop message when arriving

	rospy.spin()