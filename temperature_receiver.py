#!/usr/bin/env python 
   
import socket
import time

print("script started")

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
    print("received message: ", data.decode('UTF-8'))
    
    # short pause
    time.sleep(1)
    counter += 1