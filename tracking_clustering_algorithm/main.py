import Radar as R
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import os
import time
import argparse
import pandas as pd
import time
import socket
from threading import Thread

#currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
HOST, PORT = "192.168.88.21", 1234
clients = {}
clients_lock = Lock()
collect_radar_data = './run.sh data.csv > output.txt 2>&1'

def handle_client(client_address):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(client_address)
        while True:
            data, addr = sock.recvfrom(1024)
            imu_data = [float(val) for val in data.decode().split(',')]
            imu_frame = iframe(imu_data)
            with clients_lock:
                if client_address not in clients.keys():
                    clients[client_address] = R.client(imu_frame)
                else:
                    clients[client_address] = clients[client_address].update_imu_data(imu_frame)
def run_radar_collection():
    process = subprocess.Popen(collect_radar_data, cwd=cwd, shell = True)
    time.sleep(args.time)
    ct = time.time()
    usr = psutil.Process().username()
    terminated_pids = set()
    for proc in psutil.process_iter(['pid', 'create_time', 'username', 'cmdline']):
        if ct - proc.info['create_time'] < 5 and proc.username() == usr and proc.pid not in terminated_pids and proc.pid != os.getpid() and 'python' not in proc.info['cmdline']:
            try:
                proc.kill()
                terminated_pids.add(proc.pid)
            except psutil.NoSuchProcess:
                pass
def check_connections():
    with clients_lock:
        for client_address in clients.keys():
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                try:
                    sock.connect(client_address)
                    print("Connection to {} is still active".format(client_address))
                except:
                    print("Connection to {} has been lost".format(client_address))
frame_num = '1'
Frame_time = 0
## initialization ##
Cluster_dict = {}
Next_Cluster_dict = {}
Next_Velocity_dict = {}
## KalmanFilter for Client 1
KalmanMeasurements = np.zeros((4,1))
KalmanP = np.zeros((4,4,1))
Innovation = np.zeros((2,1))
KalmanF = np.zeros((4,2,1))
ConditionalX = np.zeros((4,1))
ConditionalP=np.identity(4)

## KalmanFilter for Client 2 

KalmanMeasurements2 = np.zeros((4,1))
KalmanP2 = np.zeros((4,4,1))
Innovation2 = np.zeros((2,1))
KalmanF2 = np.zeros((4,2,1))
ConditionalX2 = np.zeros((4,1))
ConditionalP2 = np.identity(4)

parser = argparse.ArgumentParser(description='This is a script that performs client localization and beam angle calculation based on radar and IMU data')
parser.add_argument('time', type=int, help='time in seconds to let radar run for', required = True)
args = parser.parse_args()

cwd = os.getcwd() ##
print(cwd) ## make sure we're in the right place modify accordingly
##client create
x = 0
client_ips = []
client1 = R.client()
client2 = R.client()

Start = False
for i in range(2):
    client_address = ("", PORT + i + 1)
    Thread(target = handle_client, args = (client_address,), daemon=True).start()
while(x < 5): ## get 5 global frames ## change to True later
    run_radar_collection()
    data_df = pd.read_csv('data.csv')
    nunique_frames = data_df.iloc[:,1].nunique()
    if nunique_frames < 2:
        # if we didn't get enough frames rerun data collection
        continue
    #here we can edit the csv file.
    with clients_lock:
        #if first loop, assign client 1/2's ip address
        if not Start:
            for client in clients:
                client_ips.append(client.id)

        #ensure client1 and client2 always stay the same
        client1 = clients[client_ips[0]]
        client2 = clients[client_ips[1]]

    if Start == False:
        Start_time = time.time()
    else:
        Stop_time = time.time()
        Frame_time = Stop_time - Start_time
        Start_time = time.time()
    
    
    #check what the name of this column is#
    num_frames = data_df.iloc[:,1].nunique()
    print("DEBUG: Number of unique frames: ", num_frames)
    ##TODO migrate these calls the global frame function##
        ## Get data function returns first time, last time in one global frame and frametime between microframe (if it is not the first micro frame in global frame)
    data = R.getData(data_df)

    if Start == False:
        Frame = R.rframe(frame_num,data,None,None,Start_time)
    else:
        Frame = R.rframe(frame_num,data,None,Prev_Frame,Start_time)
    
    ## Get number of clusters and labels of each point in clusters 
    labels,n_clusters_ = Frame.getCluster()
    ## Get Core points of each unique cluster
    CorePoints = Frame.getcorePoint(n_clusters_,labels)
    ## Update clusters 
    Cluster_dict, Next_Cluster_dict, Next_Velocity_dict = Frame.updatecluster(CorePoints,Next_Cluster_dict,n_clusters_,Next_Velocity_dict)

    # populate client imuFrame var with iframe from server

    R.trackOrientation6D(Frame_time,client1)
    R.trackOrientation6D(Frame_time,client2)
    R.getVelocity(Frame_time,client1,client2)
    # Update Orientation
    R.quaternion_to_euler(client1)
    R.quaternion_to_euler(client2)
    ## Identify router and return label of that router, runs every global frame
    if Start == True:
        client1_id = Frame.findrouter(client1, Next_Velocity_dict)
        client2_id = Frame.findrouter(client2, Next_Velocity_dict)

        KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP = Frame.kalmanFilter\
                (client1,Next_Cluster_dict,KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP)
        
        KalmanMeasurements2,KalmanP2,Innovation2,KalmanF2,ConditionalX2,ConditionalP2 = Frame.kalmanFilter\
                (client2,Next_Cluster_dict,KalmanMeasurements2,KalmanP2,Innovation2,KalmanF2,ConditionalX2,ConditionalP2)

        ## Get Beamforming angle
        Theta = Frame.getEstimate (client1,client2)

        Beamangle = R.beamform_angle(Theta,client1)

        #why is there only one beam angle?
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            with clients_lock:
                sock.sendto(str(Beamangle).encode(), (client_ips[0], PORT))
                sock.sendto(str(Beamangle).encode(), (client_ips[1], PORT))
                print("#### Sending to {}:{} ####".format(client_ips[0], Beamangle))
                print("#### Sending to {}:{} ####".format(client_ips[1], Beamangle))
    
    Prev_Frame = Frame
    
    frame_num += 1
    Start = True
    x += 1  
    """     fig = plt.figure()
    ax =  fig.add_subplot(projection='3d')
    
    for row in total_Cluster_Kalman:
        ax.scatter(row[0],row[1])
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    
    plt.show() """