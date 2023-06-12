#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy				
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ContactsState, ModelState, ModelStates
from geometry_msgs.msg import Twist
import numpy as np
# import torch
import pandas as pd

from std_msgs.msg import Empty as Empty_msg


import rospy
from std_srvs.srv import Empty



# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# torch.manual_seed(777)


class learning_class:
    def __init__(self):
        # Box Information
        self.Box_dict = {}
        boxname = ["box1", "box2", "box3", "box4", "box5", "box6", "box7", "box8", "box9", "box10","box_target"]
        for i in range(len(boxname)): # box1 ~ box10 init
            self.Box_dict[boxname[i]] = [0,0,0, 0,0,0] # x,y,z, rx,ry,rz
        

        # robot position
        self.nexus_x = 0
        self.nexus_y = 0
        self.nexus_z = 0

        self.nexus_rx = 0
        self.nexus_ry = 0
        self.nexus_rz = 0

        self.contact = False

        # laser
        self.laser_observation = []

        # Robot Action : [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
        self.action = [0,0,0,0,0,0]
        self.action_past = [0,0,0,0,0,0]




        rospy.Subscriber('/base_link_contact_sensor_state', ContactsState, self.callback_contact)
        rospy.Subscriber('/gazebo/set_model_state', ModelState, self.box_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.nexusPos_callback)
        rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)
        self.publish_agent_action = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


        




    # Callback Functions
    def laser_callback(self, msg):
        self.laser_observation = msg.ranges

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
        print('Contact : ',self.contact)
        print('Robot Position : ', self.x, self.y, self.z)
        print('Target_box Position : ', self.Box_dict['box_target'])
        print('laser: ', np.array(self.laser_observation).shape)

    def nexusPos_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == 'nexus_4wd_mecanum':
                self.nexus_x = msg.pose[i].position.x
                self.nexus_y = msg.pose[i].position.y
                self.nexus_z = msg.pose[i].position.z

                self.nexus_rx = msg.pose[i].orientation.x
                self.nexus_ry = msg.pose[i].orientation.y
                self.nexus_rz = msg.pose[i].orientation.z



    # Reinforcement Learning Functions
    def get_state(self):
        """
        State : Laser, Robot Position, Target Box Position
        """
        state = np.concatenate((self.laser_observation, 
                                [self.nexus_x, self.nexus_y, self.nexus_rz], 
                                self.Box_dict['box_target']), axis=0)
        return state

    def get_reward(self):
        """
        Reward Setting : Distance between Robot and Target Box
        """
        reward = 2-np.sqrt((self.nexus_x - self.Box_dict['box_target'][0])**2 + (self.nexus_y - self.Box_dict['box_target'][1])**2)
        # print('x:',self.nexus_x)
        # print('y:',self.nexus_y)

        # print('target_X:',self.Box_dict['box_target'][0])
        # print('target_Y',self.Box_dict['box_target'][1])
        print(reward)
        return reward






    # Publish Agent Action
    def pub(self):        
        pub_action = Twist()
        pub_action.linear.x = 1
        pub_action.linear.y = self.action[1]
        pub_action.linear.z = self.action[2]

        pub_action.angular.x = self.action[3]
        pub_action.angular.y = self.action[4]
        pub_action.angular.z = self.action[5]

        self.publish_agent_action.publish(pub_action)
        
    
def main():
    # initialize node
    rospy.init_node('agent', anonymous=False)
    rate = rospy.Rate(100)

    process = learning_class()
    while not rospy.is_shutdown():

        
        # If Collision, reset world
        if process.contact == True:
            rospy.wait_for_service('/gazebo/reset_world')
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()

        # process.test_callback()
        process.get_reward()
        process.pub()
        rate.sleep()








if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass