import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from gui_app_class import GUIApp #import GUIApp class for the gui elements (button etc.) and their functionalities
from services_and_messages.msg import Echosensors
from services_and_messages.msg import TOF_sensor

class Compare():

    def __init__(self):
        self.col_happend = False
        self.stair_happend = False
        self.list_col = [True,True,True,True]  
        self.list_stair = [True,True,True,True]  
        

    def give_list_col(self, list):
        self.list_col = list
        self.set_col()
    
    def give_list_stair(self, list):
        self.list_stair = list
        self.set_stair()

    def set_col(self):
        self.col_happend = True

    def set_stair(self):
        self.stair_happend = True
        

    def change_colour(self, function_handler):
        if(self.stair_happend and self.col_happend):
            bool_list = [True,True,True,True]                                 
            bool_list[0] = self.list_col[0] or self.list_stair[0]
            bool_list[1] = self.list_col[1] or self.list_stair[1]
            bool_list[2] = self.list_col[2] or self.list_stair[2]
            bool_list[3] = self.list_col[3] or self.list_stair[3]
            function_handler.change_colour_on_sensor_labels(bool_list)
            self.col_happend = False
            self.stair_happend = False
           

        