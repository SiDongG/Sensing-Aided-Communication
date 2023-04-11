import socket
from IMU import *
import Radar as R
from threading import Thread
from threading import Lock
HOST = '192.168.88.21'
PORT = 1234
clients = {}
clients_lock = Lock()
client_ips = []
Start = False
def handle_client(client_address):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(client_address)
        while True:
            data, addr = sock.recvfrom(1024)
            print(f'Received data from {addr}: {data}')
            imu_data = [float(val) for val in data.decode().split(',')]
            imu_frame = iframe(imu_data)
            #print(imu_frame)
            with clients_lock:
                if addr not in clients.keys():
                    clients[addr] = R.client(imu_frame, client_address)
                    client_ips.append(addr)
                else:
                    clients[addr] = clients[client_address].update_imu_data(imu_frame)
for i in range(2):
    client_address = ("", PORT + i + 1)
    Thread(target = handle_client, args = (client_address,), daemon=True).start()
while(True):
        #if first loop, assign client 1/2's ip address
    if not Start:
        with clients_lock:
            for ip in client_ips:
                print("client_OBJ:", clients[ip].imuFrame)
                #client_ips.append(c.id)

        #ensure client1 and client2 always stay the same
        if len(client_ips) < 2:
            #print("not connected")
            continue
        with clients_lock:
            client1 = clients[client_ips[0]]
            client2 = clients[client_ips[1]]
        
        print(f'Client 1\n{client1.imuFrame}')
        print(f'Client 2 \n{client2.imuFrame}')
        Beamangle = 20
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            with clients_lock:
                s.sendto(str(Beamangle).encode(), (client_ips[0], PORT))
                s.sendto(str(Beamangle).encode(), (client_ips[1], PORT))
            print("#### Sending to {}:{} ####".format(client_ips[0], Beamangle))
            print("#### Sending to {}:{} ####".format(client_ips[1], Beamangle))
        Start = True
