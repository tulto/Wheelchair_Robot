#!/usr/bin/env python3

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton
from kivymd.uix.dialog import MDDialog
from kivy.core.window import Window
import os
from std_msgs.msg import String


#creating a class used to make/connect the .kv file in order to run/show the gui
class GUIApp(MDApp):

    dialog = None

    def __init__(self,  **kwargs):

        super().__init__(**kwargs)
        self.screen=Builder.load_file('ros_robot_gui.kv')
        self.pub = rospy.Publisher("/nav_goal", String, queue_size=25)
        self.cancel_pub =rospy.Publisher("/nav_cancle", String, queue_size=25)
        self.msg = String()
        self.goal= ""
        self.gate= True
        self.reached=False
        

    def open_gate(self):
        self.gate=True
        self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
        self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
        self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
        self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
        self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
        self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]

    def close_gate(self):
        self.gate=False       
        
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
    
    
    def change_colour_on_sensor_labels(self, boolean_list):    #changing the colour of the labels showing in which direction the sensors sent a warning
        #self.screen gives access to the root of the .kv file (so basically to FloatLayout)
        #with the self.screen.ids.<id_name>.text="change text here"
        
        if boolean_list[0]:
            self.screen.ids.vorne_melder.md_bg_color=[1,0,0,1]
        else:
            self.screen.ids.vorne_melder.md_bg_color=[0,0.9,0.25,1]


        if boolean_list[1]:
            self.screen.ids.links_melder.md_bg_color=[1,0,0,1]
        else:
            self.screen.ids.links_melder.md_bg_color=[0,0.9,0.25,1]
            

        if boolean_list[2]:
            self.screen.ids.rechts_melder.md_bg_color=[1,0,0,1]
        else:
            self.screen.ids.rechts_melder.md_bg_color=[0,0.9,0.25,1]


        if boolean_list[3]:
            self.screen.ids.hinten_melder.md_bg_color=[1,0,0,1]
        else:
            self.screen.ids.hinten_melder.md_bg_color=[0,0.9,0.25,1]


    def pub_nav_goal_aufenthaltsraum(self, *args):
        self.msg.data = "Aufenthaltsraum"
        if self.gate==True:
            self.close_gate()
            self.goal=self.msg.data
            self.pub.publish(self.msg)
            
        

    def pub_nav_goal_cafe(self, *args):
        self.msg.data = "Cafe"
        if self.gate==True:
            self.close_gate()
            self.goal=self.msg.data
            self.pub.publish(self.msg)

    def pub_nav_goal_gruppenraum(self, *args):
        self.msg.data = "Gruppenraum"
        if self.gate==True:
            self.close_gate()
            self.goal=self.msg.data
            self.pub.publish(self.msg)

    def pub_nav_goal_ruheraum(self, *args):
        self.msg.data = "Ruheraum"
        if self.gate==True:
            self.close_gate()
            self.goal=self.msg.data
            self.pub.publish(self.msg)
    
    def pub_nav_goal_schlafzimmer(self, *args):
        self.msg.data = "Schlafzimmer"
        if self.gate==True:
            self.close_gate()
            self.goal=self.msg.data
            self.pub.publish(self.msg)
    
    def pub_nav_goal_speisesaal(self, *args):
        self.msg.data = "Speisesaal"
        if self.gate==True:
            self.close_gate()
            self.goal=self.msg.data
            self.pub.publish(self.msg)

    def pub_nav_cancel(self, *args):
        self.msg.data = "cancel_nav"
        self.open_gate()
        self.goal=self.msg.data
        self.cancel_pub.publish(self.msg)

    def temporary_button_color(self, active, *args):
        
        if active==1:
        
            if self.goal=="Aufenthaltsraum":
                self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0.75,0,0,1]
                
            
            if self.goal=="Cafe":
                self.screen.ids.Cafe_button.md_bg_color=[0.75,0,0,1]

            
            if self.goal=="Gruppenraum":
                self.screen.ids.Gruppenraum_button.md_bg_color=[0.75,0,0,1]
            
            if self.goal=="Ruheraum":
                self.screen.ids.Ruheraum_button.md_bg_color=[0.75,0,0,1]
            
            if self.goal=="Schlafzimmer":
                self.screen.ids.Schlafzimmer_button.md_bg_color=[0.75,0,0,1]
            
            if self.goal=="Speisesaal":
                self.screen.ids.Speisesaal_button.md_bg_color=[0.75,0,0,1]
                
        else: 
            self.open_gate()
            """ 
            self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
            self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
            self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
            self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
            self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
            self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]
            
                """
            
        
        
        


