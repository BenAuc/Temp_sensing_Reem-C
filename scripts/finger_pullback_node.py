#! /usr/bin/python
# -*- coding: utf-8 -*-
   

# #################################################################
# file name: object_tracker_node.py
# author's name: Diego, Benoit, Priya, Vildana
# created on: 30-05-2022
# last edit: 23-06-2022 (Benoit Auclair): removed all methods and variables unnecessary
# to the task of object tracking
# function: ROS node tracking a red blob in NAO's visual field
#################################################################
from __future__ import print_function

import argparse
import sys
import signal
from time import time, sleep
from uuid import uuid4
import rospy
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult

#####

import rospy
#from sensor_msgs.msg import Int32MultiArray
import numpy as np
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction
from actionlib import SimpleActionClient
# from move_joint import MotionData, MoveJointException
from modified_move_joint import codeCodes, printc, bold, print_err, print_ok, parse_args, wait_for_clock, play_motion_client, load_motion, unload_motion, active_cb, move_joint_cmd
from modified_move_joint import MotionData, MoveJointException

class ReemcPullback:
    """
    Class implementing the node receiving temperature readings from the Arduino
    @Inputs:
    -Temperature readings received over UDP protocole
    @Outputs:
    -publication of the temperature readings over the corresponding topic
    """

    def __init__(self):

        # define frequency of execution of this node
        self.frequency = 0.5 # Hz
        self.rate = rospy.Rate(self.frequency) # timing object

        # define subscribers
        topic_temperature_readings = "/ReemC/Fingers/Temperature"
        #rospy.Subscriber(topic_temperature_readings, Int32MultiArray, self.temperature_sub) # subscriber to NAO's camera stream

        # character separating the temperature measurements
        self.string_split = '//'

        # define topic publishers
        topic_temperature_readings = "/ReemC/Fingers/Temperature"

        # define messages
        self.temperature_readings_msg = None

        # create topic publishers
        self.right_arm_joint_pub = rospy.Publisher("/right_arm_controller/state", JointTrajectoryControllerState, queue_size=1)

        # joint names on topic
        self.joint_names = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
        'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']


    def run(self):
        """
        Main loop of class.
        Inputs:
        -self
        Outputs:
        -runs the step function.
        """

        while not rospy.is_shutdown():

            # perform step
            # self.step()
            # self.rate.sleep()
            print("iteration")
            # self.send_reemc_home()

            self.move_fingers("hand_right_index_joint", "2", "3.0")

            #############move_fingers
            # self.move_fingers(joint_name = "hand_right_index_joint", position = "1", duration = "3.0")
            # rospy.sleep(1)
            
            # self.move_fingers(joint_name = "hand_right_index_joint", position = "3", duration = "3.0")
            # rospy.sleep(1)

    
    def step(self):
        """
        Perform an iteration of blob tracking.
        Inputs:
        -self
        Outputs:
        -publishes the blob coordinates as Point() message.
        """

        pass


    def move_fingers(self, joint_name, position, duration):
        """
        Concatenate the message containing the motor command.
        Code sourced from "move_joint.py"
        Inputs:
        -joint_name, position, duration:
        Outputs:
        -client
        """     
        motion_data = MotionData(name = joint_name, pos = position, dur = duration)
        move_joint_cmd()

        # ####  
        # motion_data = MotionData(name = joint_name, pos = position, dur = duration)
        # motion_data.move_joint_cmd()



    def temperature_sub(self, data):
        """
        Handle incoming temperature data message.
        Inputs:
        -array of (x,y) coordinates of an object in the visual field.
        Outputs:
        -numpy array of normalized (x,y) coordinates.
        """
        pass


    # def send_reemc_home(self):
    #     """
    #     Send reemc back to its resting position.
    #     Inputs: None
    #     Outputs:
    #     -create and publish the message on the topic
    #     """
    #     print("creating message : ")
    #     r_arm_elbow_msg = JointTrajectoryControllerState() # declare message
    #     r_arm_elbow_msg.joint_names.append(self.joint_names[3]) # elbow pitch
    #     point = JointTrajectoryPoint()
    #     point.positions = [0]
    #     r_arm_elbow_msg.points.append(point) # elbow relaxed in vertical position
    #     print(r_arm_elbow_msg)
        
    #     # r_arm_elbow_msg = JointTrajectory() # declare message
    #     # r_arm_elbow_msg.joint_names = [self.joint_names[3]] # elbow pitch
    #     # point = JointTrajectoryPoint()
    #     # point.positions = [0]
    #     # r_arm_elbow_msg.points.append(point) # elbow relaxed in vertical position
    #     # print(r_arm_elbow_msg)


    #     self.right_arm_joint_pub.publish(r_arm_elbow_msg)
                   


if __name__=='__main__':

    #initialize the node and set name
    rospy.init_node('object_tracker',anonymous=True) #initilizes node

    # instantiate class and start loop function
    try:
        reemc = ReemcPullback()
        print("node is running")
        reemc.run()
        
    except rospy.ROSInterruptException:
        pass
