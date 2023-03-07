import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import rospy
from std_msgs.msg import String, Bool
from actionlib_msgs.msg import GoalStatusArray
from sounddevice import rec, wait
from scipy.signal import butter, lfilter
import numpy as np 



#initiate PIN setup
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def ZScore(data):
    Mean = np.mean(data)
    Std = np.std(data)
    data_ZScore = np.array((data - Mean) / Std)
    return data_ZScore

def recording(sample_rate=22050,duration= 6,rec_duration=1.5,sec_slide=0.1,word_threshold=0.5,number_index= 10):
    
    #message for mic in gui 
    mic_msg= Bool()
    mic_msg.data=True
    mic_pub.publish(mic_msg)

    print("Starting recording..")
    record = rec(int(duration*sample_rate),samplerate=sample_rate, channels=1, blocking=False)
    wait()
    print("Recording finished..")

    mic_msg.data = False  
    #rospy.sleep(1)
    mic_pub.publish(mic_msg)  

    from librosa.feature import mfcc
    from tflite_runtime.interpreter import Interpreter

    record = np.squeeze(record)

    record = butter_lowpass_filter(data=record,cutoff=10000, fs=sample_rate)

    model_path = '4_layer_globalpool.tflite'


    # Load model (interpreter)
    interpreter = Interpreter(model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()


    steps_to_make=int((duration-rec_duration)/sec_slide)



    for i in range(1, steps_to_make+1):
        
        if i==1:
            window = record[:int(sample_rate * rec_duration)]

        else:
            window=record[int(i*sec_slide*sample_rate):int((sample_rate*(rec_duration+i*sec_slide)))]

        window_mfcc=mfcc(y=window, sr=sample_rate, n_mfcc=20, n_fft=512)

        window_mfcc_z=ZScore(window_mfcc)

        window_mfcc_z = np.pad(window_mfcc_z, pad_width=((0, 0), (0, 1)), mode='constant')


        in_tensor = np.float32(window_mfcc_z.reshape(1, (window_mfcc_z.shape[0]), window_mfcc_z.shape[1], 1))

        interpreter.set_tensor(input_details[0]['index'], in_tensor)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])

        output_data= np.around(output_data[0], 2)

        if np.amax(output_data) > word_threshold:
            ##

            number_index = np.argmax(output_data)

        

    if number_index < 10: 
        list =["Aufenthaltsraum", "Cafe", "Gruppenraum", "Ruheraum", "Schlafzimmer", "Speisesaal"]
        #print(list[number_index])
        msg = String()
        msg.data = list[number_index]
        goal_pub.publish(msg)

        number_index = 10 
    else: 
        pass

def butter_lowpass(cutoff, fs, order=5):
    return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def callback_subscriber_active(msg_active):
    global button_was_pushed
    global previous_status
    global mic_stopper
    
    
    if len(msg_active.status_list) == 0:
        if (button_was_pushed):
            recording()
            button_was_pushed = False
        

    elif (msg_active.status_list[0].status == 1 and previous_status == 3) or msg_active.status_list[0].status == 1 or mic_stopper == True:
        button_was_pushed = False
        
        #print("Recognition is not possible at this moment. Navigation is active!!!")
    

    else:
        if (button_was_pushed):
            recording()


        button_was_pushed = False
    
    button_was_pushed = False

    if not len(msg_active.status_list) == 0:
        previous_status = msg_active.status_list[0].status

def timer_callback(event):
    if GPIO.input(11) == GPIO.LOW:
        global button_was_pushed
        button_was_pushed = True


def callback_stop_mic(msg_stop_mic):
    global mic_stopper

    if msg_stop_mic.data == True:
        mic_stopper = True

    if msg_stop_mic.data == False:
        mic_stopper = False

if __name__ == '__main__':

    rospy.init_node('speech_recognition_on_button_press', anonymous=False)
    
    global button_was_pushed
    button_was_pushed = False

    global previous_status
    previous_status = 0

    #microphone publisher for gui
    mic_pub = rospy.Publisher("/mic_status", Bool, queue_size=10)

     
    stop_mic = rospy.Subscriber("/stop_mic", Bool, callback_stop_mic)
    

    goal_pub = rospy.Publisher("/nav_goal", String, queue_size=25)
    nav_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, callback_subscriber_active)

    rate  = 1/15 #using a rate of 15 hz in order to look for a pressed button

    timer = rospy.Timer(rospy.Duration(rate), timer_callback)

    rospy.spin()
    timer.shutdown()

