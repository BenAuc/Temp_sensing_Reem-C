#!/usr/bin/env python 
   
import socket
import time
from tkinter.messagebox import NO
import numpy as np
import matplotlib.pyplot as plt
import time

print("script started")

# character separating the temperature measurements
string_split = '//'
string_begin_end = '??'
data = None
nb_samples = 50
nb_sensors = 3
time_series = np.zeros((nb_samples, nb_sensors))

# socket parameters of receiver
UDP_IP = "192.168.1.147" # IP of PC
UDP_PORT = 7777

# open socket on receiver's end
sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
# set 1 sec. timeout on data reception
sock.settimeout(1)

# loop until x packages are acquired just for the sake of testing
counter = 0

start = time.time()

while counter < nb_samples:
    # print("waiting for package")

    # incoming data package is received
    try:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    except socket.timeout:
        pass

    # print("timeout")

    if data is not None:
    
        # decode bytes into string and print to screen
        temp_string = data.decode('UTF-8')
        # print("received message: ", temp_string)

        # parse all temperature readings in string and print to screen
        # temp_reading_list = temp_string.split(string_split)
        temp_reading_list = temp_string.split(string_split)
        print("list: ", temp_reading_list)
        time_series[counter, :] = np.array(temp_reading_list)

        # for reading in temp_reading_list:
        #     print("reading : ", reading)
        
        # short pause
        #time.sleep(1)
        counter += 1

end = time.time()
elapsed_time = end - start
time_range = np.linspace(0, elapsed_time, num = time_series.shape[0])
data = {'temperature' : time_series, 'time' : time_range}
np.save("./data", data)

# fig, ax = plt.subplots(1,1)
# fig.set_figwidth(12)

# for sensor in range(0,nb_sensors):
#     ax.plot(time_range, time_series[:, sensor], label="sensor # " + str(sensor))
    
# ax.set_title("Temperature")
# ax.legend(loc='upper right')
# ax.set_xlabel("time (s)")
# ax.set_ylabel("Temperature (C)")
# plt.show

# time.sleep(5)