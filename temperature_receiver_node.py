#! /usr/bin/python 
   
# #################################################################
# file name: temperature_receiver_node.py
# author's name: Benoit Auclair
# created on: 07-07-2022
# last edit: 13-07-2022
# function: ROS node receiving temperature readings from the Arduino
#################################################################

import rospy
from sensor_msgs.msg import Int32MultiArray
import numpy as np
import socket
import time
from control_msgs.msg import JointTrajectoryControllerState


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
        self.frequency = 0.2 # Hz
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

        # create topic publishers
        self.right_arm_joint_pub = rospy.Publisher("/right_arm_controller/state", JointTrajectoryControllerState, queue_size=1)

        # joint names on topic
        self.joint_names = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
        'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']


    def send_reemc_home(self):
        """
        Send reemc back to its resting position.
        Inputs: None
        Outputs:
        -create and publish the message on the topic
        """

        r_arm_elbow_msg = JointTrajectoryControllerState() # declare message
        r_arm_elbow_msg.joint_names = [self.joint_names[3]] # elbow pitch
        r_arm_elbow_msg.actual.positions = [0] # elbow relaxed in vertical position
        self.right_arm_joint_pub(r_arm_elbow_msg)


    def grasp_object(self):
        """
        Handle incoming data packages transmitted over UDP protocole
        Inputs: none
        Outputs:
        -create the message to be published
        """
        r_arm_elbow_msg = JointTrajectoryControllerState() # declare message
        r_arm_elbow_msg.joint_names = [self.joint_names[0], self.joint_names[5]] # shoulder pitch
        r_arm_elbow_msg.actual.positions = [0.7, 0.61] # elbow in half pent position
        self.right_arm_joint_pub(r_arm_elbow_msg)
        rospy.sleep(0.25)

        r_arm_elbow_msg = JointTrajectoryControllerState() # declare message
        r_arm_elbow_msg.joint_names = [self.joint_names[2], self.joint_names[3]] # elbow pitch
        r_arm_elbow_msg.actual.positions = [-1.53, 0.72] # elbow in half pent position
        self.right_arm_joint_pub(r_arm_elbow_msg)
        rospy.sleep(0.25)

    
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

            self.send_reemc_home()

            # sleep to target frequency
            # self.rate.sleep()
            rospy.sleep(1)

            self.grasp_object()

            rospy.sleep(1)

    
    def step(self):
        """
        Perform an iteration of object grasping.
        Inputs:
        -none
        Outputs:
        -none
        """

        # self.temperature_receiver()

        # if self.temperature_readings_msg is not None:

        #     # Publish temperature readings
        #     self.temperature_readings_msg.publish(self.temperature_readings_msg)

            
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

            print("reading : ", reading)
                self.temperature_readings_msg.append(reading) 


if __name__=='__main__':

    #initialize the node and set name
    rospy.init_node('TempReceiverNode',anonymous=True) #initilizes node

    # instantiate class and start loop function
    try:
        reemc = TemperatureReceiver()
        reemc.run()
        
    except rospy.ROSInterruptException:
        pass
