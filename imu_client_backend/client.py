import socket
import sys
import IMU
import time

HOST, PORT = "192.168.88.20", 1234
imu_up = launch_imu()
data = get_imu_data()

# Create a socket (SOCK_DGRAM means a UDP socket)
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    # Connect to server and send data
    if imu_up:
        while True:
            imu_data = get_imu_data()
            if imu_data is not None:
                #convert the list to a string and encode it
                data_str = ",".join(str(val) for val in imu_data)
                data_bytes = str.encode(data_str)
                sock.sendto(sdata_bytes, (HOST,PORT))
                
                print("#### SENDING: {}} ####".format(data_str))
                time.sleep(1)

