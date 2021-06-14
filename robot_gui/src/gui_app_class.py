#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton
from kivymd.uix.dialog import MDDialog
from kivy.core.window import Window
import os
from services_and_messages.srv import WarningEchoTest

#creating a class used to make/connect the .kv file in order to run/show the gui
class GUIApp(MDApp):

    dialog = None

    def __init__(self, **kwargs):

        self.warning_request_button_already_pressed = [False, False, False, False]
        super().__init__(**kwargs)
        self.screen=Builder.load_file('ros_robot_gui.kv')
        #if we had a second .kv file we would add it like this:
        #self.screen=Builder.load_file('/home/wheelchair-robot_catkin/src/wheelchair_robot/robot_gui/ros_robot_gui2.kv')

    def build(self):
        Window.size = (800,480)
        return self.screen

    #functions like a button press will be defined in this class (see below)
    
    def restart_pi(self, inst):
        os.system('systemctl reboot -i') #reboots system on button press


    def closeDialog(self, inst):
        self.dialog.dismiss()

    def show_alert_dialog(self): #showing pop-up window
        if not self.dialog:
            self.dialog = MDDialog(
                size_hint=[0.4, 0.3],
                title="Linux Betriebssystem Neustarten?",
                text="Soll mit dem Neustarten des Betriebssystems fortgefahren werden?\n\nACHTUNG:\nFalls Sie den Computer neustarten warten Sie bitte bis dieser wieder hochgefahren ist!",
                buttons=[
                    MDRectangleFlatButton(
                        text="Ja", text_color=self.theme_cls.primary_color, on_press=self.restart_pi
                    ),
                    MDRectangleFlatButton(
                        text="Nein", text_color=self.theme_cls.primary_color, on_release=self.closeDialog
                    ),
                ],
            )
        self.dialog.set_normal_height()
        self.dialog.open()
    
    
    def request_echo_warning_front(self, *args):
        self.change_colour_on_sensor_request_button(self.execute_warning_request("ef"))

    def request_echo_warning_left(self, *args):
        self.change_colour_on_sensor_request_button(self.execute_warning_request("el")) 

    def request_echo_warning_right(self, *args):
        self.change_colour_on_sensor_request_button(self.execute_warning_request("er"))

    def request_echo_warning_back(self, *args):
        self.change_colour_on_sensor_request_button(self.execute_warning_request("eb"))


    def execute_warning_request(self, warning_dir): #function to launch a warning request to the echo_test_warning server
        #this is only needed for test purposes
        service_acknowledge_received = False
        try:
            warning_request = rospy.ServiceProxy("/echo_test_warning", WarningEchoTest)
            response = warning_request(str(warning_dir))
            if response.acknowledge:
                rospy.loginfo("Echo-Warning-Request was succesful.")
                service_acknowledge_received = True
            else:
                rospy.loginfo("Echo-Warning-Request was not successful.")
                service_acknowledge_received = False
        except rospy.ServiceException as e:
            rospy.logwarn("Service failed: " + str(e))
        
        if service_acknowledge_received:
            if warning_dir == "ef":
                if not self.warning_request_button_already_pressed[0]:
                    self.warning_request_button_already_pressed[0] = True
                else:
                    self.warning_request_button_already_pressed[0] = False
            if warning_dir == "el":
                if not self.warning_request_button_already_pressed[1]:
                    self.warning_request_button_already_pressed[1] = True
                else:
                    self.warning_request_button_already_pressed[1] = False
            if warning_dir == "er":
                if not self.warning_request_button_already_pressed[2]:
                    self.warning_request_button_already_pressed[2] = True
                else:
                    self.warning_request_button_already_pressed[2] = False
            if warning_dir == "eb":
                if not self.warning_request_button_already_pressed[3]:
                    self.warning_request_button_already_pressed[3] = True
                else:
                    self.warning_request_button_already_pressed[3] = False

        return self.warning_request_button_already_pressed


    def change_colour_on_sensor_labels(self, boolean_list):    #changing the colour of the labels showing in which direction the sensors sent a warning
        #self.screen gives access to the root of the .kv file (so basically to FloatLayout)
        #with the self.screen.ids.<id_name>.text="change text here"
        
        if boolean_list[0]:
            self.screen.ids.front_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.front_sensor_label.text="Blockiert"
        else:
            self.screen.ids.front_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.front_sensor_label.text="Frei"


        if boolean_list[1]:
            self.screen.ids.left_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.left_sensor_label.text="Blockiert"
        else:
            self.screen.ids.left_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.left_sensor_label.text="Frei"
            

        if boolean_list[2]:
            self.screen.ids.right_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.right_sensor_label.text="Blockiert"
        else:
            self.screen.ids.right_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.right_sensor_label.text="Frei"


        if boolean_list[3]:
            self.screen.ids.back_sensor.md_bg_color=[1,0,0,1]
            self.screen.ids.back_sensor_label.text="Blockiert"
        else:
            self.screen.ids.back_sensor.md_bg_color=[0,1,0,1]
            self.screen.ids.back_sensor_label.text="Frei"


    def change_colour_on_sensor_request_button(self, boolean_button_value):
        #this function changes the colour of the buttons, which are used to request a warning
        if not boolean_button_value[0]:
            self.screen.ids.front_warning.md_bg_color=[0,0,1,1]
        else:
            self.screen.ids.front_warning.md_bg_color=[1,0,0,1]
        
        if not boolean_button_value[1]:
            self.screen.ids.left_warning.md_bg_color=[0,0,1,1]
        else:
            self.screen.ids.left_warning.md_bg_color=[1,0,0,1]

        if not boolean_button_value[2]:
            self.screen.ids.right_warning.md_bg_color=[0,0,1,1]
        else:
            self.screen.ids.right_warning.md_bg_color=[1,0,0,1]

        if not boolean_button_value[3]:
            self.screen.ids.back_warning.md_bg_color=[0,0,1,1]
        else:
            self.screen.ids.back_warning.md_bg_color=[1,0,0,1]

        '''
        if not boolean_button_value:
            self.screen.ids.front_warning.md_bg_color=[0,0,1,1]
            self.screen.ids.left_warning.md_bg_color=[0,0,1,1]
            self.screen.ids.right_warning.md_bg_color=[0,0,1,1]
            self.screen.ids.back_warning.md_bg_color=[0,0,1,1]
        else:
            if warning_dir == "ef":
                self.screen.ids.front_warning.md_bg_color=[1,0,0,1]
            if warning_dir == "el":
                self.screen.ids.left_warning.md_bg_color=[1,0,0,1]
            if warning_dir == "er":
                self.screen.ids.right_warning.md_bg_color=[1,0,0,1]
            if warning_dir == "eb":
                self.screen.ids.back_warning.md_bg_color=[1,0,0,1]
        '''
            


        


