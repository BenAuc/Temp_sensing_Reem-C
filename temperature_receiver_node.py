#!/usr/bin/env python 
   
# #################################################################
# file name: temperature_receiver_node.py
# author's name: Benoit Auclair
# created on: 07-07-2022
# last edit: 13-07-2022 (Benoit Auclair): added publisher
# function: ROS node receiving temperature readings from the Arduino
#################################################################

import rospy
from sensor_msgs.msg import Int32MultiArray
import numpy as np
import socket
import time


class TemperatureReceiver:
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

        # socket parameters of receiver
        self.UDP_IP = "192.168.1.147" # IP of PC
        self.UDP_PORT = 8888

        # open socket on receiver's end
        self.sock = socket.socket(socket.AF_INET, # Internet
                                socket.SOCK_DGRAM) # UDP
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        
        # set timeout on data reception
        self.socket_delay = 1 # seconds
        self.sock.settimeout(self.socket_delay)

        # character separating the temperature measurements
        self.string_split = '//'

        # define topic publishers
        topic_temperature_readings = "/ReemC/Fingers/Temperature"
        self.redBlobPub = rospy.Publisher(topic_temperature_readings, Int32MultiArray, queue_size=1)  # the centroid coordinates of the tracked object

        # define messages
        self.temperature_readings_msg = None


    def temperature_receiver(self):
        """
        Handle incoming data packages transmitted over UDP protocole
        Inputs: none
        Outputs:
        -create the message to be published
        """
        
        print("waiting for package")

        data = None

        # incoming data package is received
        try:
            data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
        except socket.timeout:
            pass

        print("timeout")

        if data is not None:

            self.temperature_readings_msg = Int32MultiArray()
        
            # decode bytes into string and print to screen
            temp_string = data.decode('UTF-8')
            print("received message: ", temp_string)

            # parse all temperature readings in string and print to screen
            temp_reading_list = temp_string.split(string_split)
            print("list: ", temp_reading_list)

            print("type :", type(int(temp_reading_list[0])))

            for reading in temp_reading_list:
                print("reading : ", reading)
                self.temperature_readings_msg.append(reading)
        
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

        self.temperature_receiver()

        if self.temperature_readings_msg is not None:

            # Publish temperature readings
            self.temperature_readings_msg.publish(self.temperature_readings_msg)
            

if __name__=='__main__':

    #initialize the node and set name
    rospy.init_node('temperature_receiver',anonymous=True) #initilizes node

    # instantiate class and start loop function
    try:
        object_tracker = TemperatureReceiver()
        object_tracker.run()
        
    except rospy.ROSInterruptException:
        pass
