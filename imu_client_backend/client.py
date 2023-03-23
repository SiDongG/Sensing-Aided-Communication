import socket
import sys
from IMU import *
import time

HOST, PORT = "192.168.88.21", 1234
data = get_imu_data()

# Create a socket (SOCK_DGRAM means a UDP socket)
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    # Connect to server and send data
    while True:
        imu_data = get_imu_data()
        if imu_data is not None:
            #convert the list to a string and encode it
            data_str = ",".join(str(val) for val in imu_data)
            data_bytes = str.encode(data_str)
            sock.sendto(data_bytes, (HOST,PORT))
            
            print("#### SENDING: {} ####".format(data_str))
            time.sleep(1)   
