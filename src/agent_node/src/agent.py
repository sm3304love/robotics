#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy				
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState, ModelState
from geometry_msgs.msg import Twist
import numpy as np
import torch
import pandas as pd


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
torch.manual_seed(777)


class learning_class:
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        rospy.Subscriber('/base_link_contact_sensor_state', ContactsState, self.callback_contact)
        rospy.Subscriber('/gazebo/set_model_state', ModelState, self.box_callback)
        self.publish_agent_action = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # robot position
        self.x = 0
        self.y = 0
        self.z = 0

        self.rx = 0
        self.ry = 0
        self.rz = 0

        # Box Information
        self.Box_dict = {}
        boxname = ["box1", "box2", "box3", "box4", "box5", "box6", "box7", "box8", "box9", "box10"]
        for i in range(10): # box1 ~ box10 init
            self.Box_dict[boxname[i]] = [0,0,0, 0,0,0] # x,y,z, rx,ry,rz

        self.contact = False


    def callback_odom(self, msg):
        self.x =  msg.pose.pose.position.x
        self.y =  msg.pose.pose.position.y
        self.z =  msg.pose.pose.position.z

        self.rx = msg.pose.pose.orientation.x
        self.ry = msg.pose.pose.orientation.y
        self.rz = msg.pose.pose.orientation.z

    def box_callback(self, msg):
        box_name = msg.model_name
        box_pose = msg.pose
        box_position = box_pose.position
        box_orientation = box_pose.orientation

        self.Box_dict[box_name] = [box_position.x, box_position.y, box_position.z, box_orientation.x, box_orientation.y, box_orientation.z]
        

    def callback_contact(self, msg):
        if len(msg.states) > 0:
            self.contact = True
        else:
            self.contact = False
        

    def test_callback(self):
<<<<<<< HEAD
        print(self.contact)
        print(self.x)
        print(self.Box_dict["box1"])
=======
        print('Contact : ',self.contact)
        print('Robot Position : ', self.x, self.y, self.z)
>>>>>>> main

    def pub(self):        
        pub_action = Twist()
        pub_action.linear.x = 0.5
        pub_action.linear.y = 0.5
        pub_action.linear.z = 0.0

        pub_action.angular.x = 0.0
        pub_action.angular.y = 0.0
        pub_action.angular.z = 0.3

        self.publish_agent_action.publish(pub_action)

        
    
def main():
    # initialize node
    rospy.init_node('imu_sensor_collect', anonymous=False)
    rate = rospy.Rate(100)

    process = learning_class()
    while not rospy.is_shutdown():

        process.test_callback()
        process.pub()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass