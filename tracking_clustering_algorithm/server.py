import socket
import IMU

HOST = '192.168.88.20'
PORT = 1234

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind((HOST,PORT))
    print(f'Listening on {HOST}:{PORT}...')
    while True:
        data, addr = sock.recvfrom(1024)
        imu_data = [float(x) for x in data.decode().split(',')]
        this_frame = iframe(imu_data)
        print(f'Received IMU data from {addr}: \n {this_frame}')