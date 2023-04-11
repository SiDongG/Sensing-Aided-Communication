import socket
import sys
import IMU as i
import time

HOST = "192.168.88.21"
SERVER_PORT = 1234

# assign a unique port number for this client based on its index
client_index = 0  # change this to 1 for the second client
PORT = SERVER_PORT + client_index + 1

data = i.get_imu_data()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))
# Create a socket (SOCK_DGRAM means a UDP socket)
# Connect to server and send data
while True:
    imu_data = i.get_imu_data()
    if imu_data is not None:
        #convert the list to a string and encode it
        data_str = ",".join(str(val) for val in imu_data)
        data_bytes = str.encode(data_str)
        sock.sendto(data_bytes, (HOST,PORT))
        
        print("#### SENDING: {} ####".format(data_str))
    data, server_address = sock.recvfrom(1024)
    beamangle = data.decode()
    print("#### RECEIVED FROM {}: {} ####".format(server_address, beamangle))
    time.sleep(1)   
