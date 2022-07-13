#!/usr/bin/env python 
   
import socket
import time
from tkinter.messagebox import NO

print("script started")

# character separating the temperature measurements
string_split = '//'
data = None

# socket parameters of receiver
UDP_IP = "192.168.1.147" # IP of PC
UDP_PORT = 8888

# open socket on receiver's end
sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
# set 1 sec. timeout on data reception
sock.settimeout(1)

# loop until x packages are acquired just for the sake of testing
counter = 0
while counter != 15:
    print("waiting for package")

    # incoming data package is received
    try:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    except socket.timeout:
        pass

    print("timeout")

    if data is not None:
    
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