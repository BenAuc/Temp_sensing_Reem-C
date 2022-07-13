#!/usr/bin/env python 
   
import socket
import time

print("script started")

# character separating the temperature measurements
string_split = '//'

# socket parameters of receiver
UDP_IP = "192.168.1.147" # IP of PC
UDP_PORT = 8888

# open socket on receiver's end
sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

# loop until x packages are acquired just for the sake of testing
counter = 0
while counter != 15:
    print("waiting for package")

    # incoming data package is received
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes

    # decode bytes into string and print to screen
    temp_string = data.decode('UTF-8')
    print("received message: ", temp_string)

    # parse all temperature readings in string and print to screen
    temp_reading_list = temp_string.split(string_split)
    print("list: ", temp_reading_list)

    print("type :", type(int(temp_reading_list[0])))

    for reading in temp_reading_list:
        print("reading : ", reading)
    
    # short pause
    time.sleep(1)
    counter += 1


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


class ObjectTracker:
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

        # define topic publishers
        topic_temperature_readings = "/ReemC/Fingers/Temperature"
        self.redBlobPub = rospy.Publisher(topic_temperature_readings, Int32MultiArray, queue_size=1)  # the centroid coordinates of the tracked object

        # define messages
        self.temperature_readings_msg = None


    def temperature_receiver(self, coordinates):
        """
        Normalize the blob coordinates.
        Inputs:
        -array of (x,y) coordinates of an object in the visual field.
        Outputs:
        -numpy array of normalized (x,y) coordinates.
        """

        return np.array(coordinates, dtype=float) / np.array([self.cam_x_max, self.cam_y_max], dtype=float)


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
