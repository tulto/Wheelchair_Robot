#!/usr/bin/env python3

from curses.textpad import rectangle
from tkinter import Button
import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivymd.uix.dialog import MDDialog
from kivy.core.window import Window
import os
from kivymd.uix.button import MDFlatButton
from std_srvs.srv import Empty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from std_msgs.msg import String, Bool
()
from kivymd.uix.list import OneLineAvatarIconListItem
from kivy.properties import StringProperty
from kivy.uix.behaviors import ButtonBehavior

###
class ItemConfirm(OneLineAvatarIconListItem):
    divider = None

    global kill #does not work :)
    kill = False

    def __init__(self, name,  **kwargs):
	
        super().__init__(**kwargs)
        self.pub_save_pos = rospy.Publisher("/nav_save_position", String, queue_size=25)
        self.name = name
        


    def on_press(self):
        msg_save_pos = String()
        if self.name == "Aufenthaltsraum":
            msg_save_pos.data = self.name
            self.pub_save_pos.publish(msg_save_pos)
            kill = True

        if self.name == "Cafe":
            msg_save_pos.data = self.name
            self.pub_save_pos.publish(msg_save_pos)
            kill = True

        if self.name == "Gruppenraum":
            msg_save_pos.data = self.name
            self.pub_save_pos.publish(msg_save_pos)
            kill = True

        if self.name == "Ruheraum":
            msg_save_pos.data = self.name
            self.pub_save_pos.publish(msg_save_pos)
            kill = True

        if self.name == "Schlafzimmer":
            msg_save_pos.data = self.name
            self.pub_save_pos.publish(msg_save_pos)
            kill = True

        if self.name == "Speisesaal":
            msg_save_pos.data = self.name
            self.pub_save_pos.publish(msg_save_pos)
            kill = True

        
        return super().on_press()

    def set_icon(self, instance_check):
        instance_check.active =True

        

        """
        check_list = instance_check.get_widgets(instance_check.group)
        for check in check_list:
            if check != instance_check:
                print("test")
                check.active = False"""


#creating a class used to make/connect the .kv file in order to run/show the gui
class GUIApp(MDApp):

    dialog = None

    def __init__(self,  **kwargs):

        super().__init__(**kwargs)
        self.screen=Builder.load_file('ros_robot_gui.kv')
        self.pub = rospy.Publisher("/nav_goal", String, queue_size=25)
        self.cancel_pub =rospy.Publisher("/nav_cancel", String, queue_size=25)
        self.pub_stop_mic = rospy.Publisher("/stop_mic", Bool, queue_size=10) #Zum ausschalten des mikrofons nach dem ein button gedrückt wurde
        self.msg = String()
        self.goal= ""
        self.gate= True
        self.reached=False
        self.last_status=3

        box = BoxLayout(orientation = 'vertical', padding = (70))
        box.add_widget(Label(text="Bitte verfahren Sie den Roboter so lange manuell, \n         bis sich dieses Fenster wieder schließt!\n\n\n", font_size=32)
        , index=0)
        

        #innerbox =BoxLayout(orientation='vertical')
        btn1 = Button(text='             Lokalisation trotz mehrminütigem Verfahren nicht möglich?\nZum Neustarten der vollständigen Lokalisation bitte diesen Button drücken!', 
        size_hint=(1, 1), font_size=18)
        
        #innerbox.add_widget(btn1)
        box.add_widget(btn1,)

        
        #box.add_widget(innerbox)
        #box.bind(center=btn1.setter("center"))


        self.popup = Popup(title="Roboter nicht lokalisiert!",
        
        content=box,
         auto_dismiss=False, title_align="center", title_size=48, title_font="data/fonts/Roboto-Bold.ttf")
         
        
    
        btn1.bind(on_press = self.new_global_localization)

    def new_global_localization(self, *args):

        rospy.wait_for_service("global_localization", timeout=2.0)
        try:
            global_loc_srv = rospy.ServiceProxy('global_localization', Empty)
            call_global_loc = global_loc_srv()
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        
    def kill_dialog(self):
        if kill == True:
            self.closeDialog
            kill == False

    #
    def show_confirmation_dialog(self):
        #if not self.dialog:
        self.dialog = MDDialog(
            title="Wählen Sie eine Lokalität aus, welche Sie festlegen wollen:",
            type="confirmation",
            items=[
                ItemConfirm(text="Aufenthaltsraum", name="Aufenthaltsraum"),
                ItemConfirm(text="Café", name="Cafe"),
                ItemConfirm(text="Gruppenraum", name="Gruppenraum"),
                ItemConfirm(text="Ruheraum", name="Ruheraum"),
                ItemConfirm(text="Schlafzimmer", name="Schlafzimmer"),
                ItemConfirm(text="Speisesaal", name="Speisesaal"),
                
                
                
                
            ],
            #buttons=[MDRectangleFlatButton(text="FERTIG", text_color=self.theme_cls.primary_color, on_press=self.closeDialog),],
        )
        self.kill_dialog
        self.dialog.set_normal_height()
        self.dialog.open()
        #self.kill_dialog

        ##
    def set_goal(self, msg):
        self.goal = msg

        global stop_mic_msg #Zum ausschalten des mikrofons nach dem ein button gedrückt wurde ansonsten dauert es zu lange wegen move base status 
        stop_mic_msg= Bool()
        
        
        if self.goal=="Aufenthaltsraum":
            self.close_gate()
            self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0.75,0,0,1]
            stop_mic_msg.data=True #Zum ausschalten des mikrofons nach dem ein button gedrückt wurde
            self.pub_stop_mic.publish(stop_mic_msg)
        
        
        if self.goal=="Cafe":
            self.close_gate()
            self.screen.ids.Cafe_button.md_bg_color=[0.75,0,0,1]
        
        if self.goal=="Gruppenraum":
            self.close_gate()
            self.screen.ids.Gruppenraum_button.md_bg_color=[0.75,0,0,1]

        if self.goal=="Ruheraum":
            self.close_gate()
            self.screen.ids.Ruheraum_button.md_bg_color=[0.75,0,0,1]

        if self.goal=="Schlafzimmer":
            self.close_gate()
            self.screen.ids.Schlafzimmer_button.md_bg_color=[0.75,0,0,1]

        if self.goal=="Speisesaal":
            self.close_gate()
            self.screen.ids.Speisesaal_button.md_bg_color=[0.75,0,0,1]

            

        

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
        #if not self.dialog:
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


    def pop_up_localization(self, pop):

        if pop:
            self.popup.open()
        else:
            #print("test")
            self.popup.dismiss(force=True, animation=False)

            

    
    
    
        
    def mic_color_change(self, activate):
        if activate:
            self.screen.ids.mic_status.md_bg_color=[0,0.9,0.25, 1]
            self.screen.ids.mic_text.text_color = [0, 0, 0, 1]
            self.screen.ids.mic_text.text= "  Mikrofon \n  aktiviert"

        else:
            self.screen.ids.mic_status.md_bg_color=[0.75, 0, 0, 1]
            self.screen.ids.mic_text.text_color = [1, 1, 1, 1]
            self.screen.ids.mic_text.text ="  Mikrofon \n  deaktiviert"

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
            self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0.75,0,0,1]
            self.pub.publish(self.msg)
            

    def pub_nav_goal_cafe(self, *args):
        self.msg.data = "Cafe"
        if self.gate==True:
            self.close_gate()
            self.screen.ids.Cafe_button.md_bg_color=[0.75,0,0,1]
            self.pub.publish(self.msg)

    def pub_nav_goal_gruppenraum(self, *args):
        self.msg.data = "Gruppenraum"
        if self.gate==True:
            self.close_gate()
            self.screen.ids.Gruppenraum_button.md_bg_color=[0.75,0,0,1]
            self.pub.publish(self.msg)

    def pub_nav_goal_ruheraum(self, *args):
        self.msg.data = "Ruheraum"
        if self.gate==True:
            self.close_gate()
            self.screen.ids.Ruheraum_button.md_bg_color=[0.75,0,0,1]
            self.pub.publish(self.msg)
    
    def pub_nav_goal_schlafzimmer(self, *args):
        self.msg.data = "Schlafzimmer"
        if self.gate==True:
            self.close_gate()
            self.screen.ids.Schlafzimmer_button.md_bg_color=[0.75,0,0,1]
            self.pub.publish(self.msg)
    
    def pub_nav_goal_speisesaal(self, *args):
        self.msg.data = "Speisesaal"
        if self.gate==True:
            self.close_gate()
            self.screen.ids.Speisesaal_button.md_bg_color=[0.75,0,0,1]
            self.pub.publish(self.msg)

    def pub_nav_cancel(self, *args):
        self.msg.data = "cancel_nav"
        self.open_gate()
        self.cancel_pub.publish(self.msg)

    def temporary_button_color(self, active, *args):
        global stop_mic_msg

        if active==1:
        
            if self.goal=="Aufenthaltsraum":
                self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0.75,0,0,1]
                #self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]
                    
            
            if self.goal=="Cafe":
                self.screen.ids.Cafe_button.md_bg_color=[0.75,0,0,1]
                self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
                #self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]

            
            if self.goal=="Gruppenraum":
                self.screen.ids.Gruppenraum_button.md_bg_color=[0.75,0,0,1]
                self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
                #self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]
            
            if self.goal=="Ruheraum":
                self.screen.ids.Ruheraum_button.md_bg_color=[0.75,0,0,1]
                self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
                #self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]
            
            if self.goal=="Schlafzimmer":
                self.screen.ids.Schlafzimmer_button.md_bg_color=[0.75,0,0,1]
                self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
                #self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]
            
            if self.goal=="Speisesaal":
                self.screen.ids.Speisesaal_button.md_bg_color=[0.75,0,0,1]
                self.screen.ids.Aufenthaltsraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Cafe_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Gruppenraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Ruheraum_button.md_bg_color=[0,0,1,1]
                self.screen.ids.Schlafzimmer_button.md_bg_color=[0,0,1,1]
                #self.screen.ids.Speisesaal_button.md_bg_color=[0,0,1,1]


        if active == 3 and not self.last_status == 3: #erst wenn flanke (von 1 auf 3 in move_base) erreich werden die Buttons wieder blau
            self.open_gate()
            stop_mic_msg.data=False
            self.pub_stop_mic.publish(stop_mic_msg)
    
        # setzt unergründlicher Weise die Buttons ständig zurück deswegen wird es weggelassen
        #if active == 2 and not self.last_status == 2:  
      
            #self.open_gate()                           
        
        if active == 4 and self.last_status== 4:
            self.open_gate()

        if active == 5 and self.last_status == 5:
            self.open_gate()

        if active == 6 and self.last_status == 6:
            self.open_gate()

        if active == 7 and self.last_status == 7:
            self.open_gate()

        if active == 8 and self.last_status == 8:
            self.open_gate()

        self.last_status = active 
            


        


