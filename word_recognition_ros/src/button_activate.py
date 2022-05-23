import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray

#initiate PIN setup
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)



def callback_subscriber_active(msg_active):
    global button_was_pushed
    if msg_active.status_list[0].status == 1:
        button_was_pushed = False
        print("Recognition is not possible at this moment. Navigation is active!!!")
    else:
        if (button_was_pushed):
            #code for speech recognition
            print("hello")
        button_was_pushed = False
    
    button_was_pushed = False

def timer_callback(event):
    if GPIO.input(11) == GPIO.LOW:
        global button_was_pushed
        button_was_pushed = True

if __name__ == '__main__':
    rospy.init_node('speech_recognition_on_button_press', anonymous=False)

    global button_was_pushed
    button_was_pushed = False

    goal_pub = rospy.Publisher("/nav_goal", String, queue_size=25)
    nav_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, callback_subscriber_active)

    rate  = 1/15 #using a rate of 15 hz in order to look for a pressed button

    timer = rospy.Timer(rospy.Duration(rate), timer_callback)

    rospy.spin()
    timer.shutdown()