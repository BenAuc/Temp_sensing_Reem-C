#!/usr/bin/env python 
   
import socket
from tkinter.messagebox import NO
import numpy as np
import matplotlib.pyplot as plt
import time

print("script started")

# character separating the temperature measurements
string_split = '//'
string_begin_end = '??'
data = None
nb_samples = 10000
nb_sensors = 3
time_series = np.zeros((10000, nb_sensors))
times = np.zeros(10000)
acquisition_time = 75 # seconds
dt = acquisition_time / nb_samples

# socket parameters of receiver
UDP_IP = "192.168.1.148" # IP of PC
UDP_PORT = 7777

# open socket on receiver's end
sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
# set 1 sec. timeout on data reception
sock.settimeout(2)

# loop until x packages are acquired just for the sake of testing
counter = 0

start = time.time()

while True:
    # print("waiting for package")
    data=None
    # incoming data package is received
    try:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    except socket.timeout:
        pass

    delta = time.time() - start
    print("end - start :", delta)
    if (delta> acquisition_time):
        break
        print("timeout")

    if data is not None:
    
        # decode bytes into string and print to screen
        temp_string = data.decode('UTF-8')
        # print("received message: ", temp_string)

        # parse all temperature readings in string and print to screen
        # temp_reading_list = temp_string.split(string_split)
        temp_reading_list = temp_string.split(string_split)
        print("list: ", temp_reading_list)
        time_series[counter, :] = np.array(temp_reading_list)
        times[counter] = delta

        counter+=1

        # for reading in temp_reading_list:
        #     print("reading : ", reading)
        
        # short pause
        time.sleep(dt)


elapsed_time = delta
print("elapsed_time :", elapsed_time)
time_range = np.linspace(0, elapsed_time, counter)
data = time_series

np.save("./data.npy", data[:counter,:])
np.save("./time_range.npy", time_range)

