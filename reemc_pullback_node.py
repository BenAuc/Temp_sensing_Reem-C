#!/usr/bin/env python 
   

# #################################################################
# file name: object_tracker_node.py
# author's name: Diego, Benoit, Priya, Vildana
# created on: 30-05-2022
# last edit: 23-06-2022 (Benoit Auclair): removed all methods and variables unnecessary
# to the task of object tracking
# function: ROS node tracking a red blob in NAO's visual field
#################################################################

import rospy
from sensor_msgs.msg import Int32MultiArray
import numpy as np


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
        self.frequency = 5.0 # Hz
        self.rate = rospy.Rate(self.frequency) # timing object

        # define subscribers
        topic_temperature_readings = "/ReemC/Fingers/Temperature"
        rospy.Subscriber(topic_temperature_readings, Int32MultiArray, self.temperature_sub) # subscriber to NAO's camera stream


    def temperature_sub(self, data):
        """
        Normalize the blob coordinates.
        Inputs:
        -array of (x,y) coordinates of an object in the visual field.
        Outputs:
        -numpy array of normalized (x,y) coordinates.
        """

        return 
        

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
            self.step()

            # sleep to target frequency
            self.rate.sleep()

    
    def step(self):
        """
        Perform an iteration of blob tracking.
        Inputs:
        -self
        Outputs:
        -publishes the blob coordinates as Point() message.
        """

        if self.temperature_readings_msg is not None:

            # Publish blob coordinates
            self.temperature_readings_msg.publish(self.blob_coordinates_msg)
            


if __name__=='__main__':

    #initialize the node and set name
    rospy.init_node('object_tracker',anonymous=True) #initilizes node

    # instantiate class and start loop function
    try:
        object_tracker = ObjectTracker()
        object_tracker.run()
        
    except rospy.ROSInterruptException:
        pass
