import Radar as R
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import os
import time
import argparse
import math
import pandas as pd
import time
import socket
from threading import Thread
from threading import Lock
from IMU import *
import psutil
from matplotlib.patches import Ellipse

#currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
HOST, PORT = "192.168.88.21", 1234
clients = {}
clients_lock = Lock()
collect_radar_data = './run.sh test.csv > output.txt 2>&1'

def handle_client(client_address, clients_dict, lock, ips_list):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(client_address)
        while True:
            data, addr = sock.recvfrom(1024)
            print(f'Received data from {addr}: {data}')
            imu_data = [float(val) for val in data.decode().split(',')]
            imu_frame = iframe(imu_data)
            #print(imu_frame)
            with lock:
                #print("in thread... ", clients_dict)
                if addr not in clients_dict.keys():
                    clients_dict[addr] = R.client(imu_frame, addr)
                    ips_list.append(addr)
                    print(ips_list)
                else:
                    clients_dict[addr].update_imu_data(imu_frame)
def run_radar_collection():
    process = subprocess.Popen(collect_radar_data, cwd=cwd, shell = True)
    time.sleep(args.time)
    ct = time.time()
    usr = psutil.Process().username()
    terminated_pids = set()
    for proc in psutil.process_iter(['pid', 'create_time', 'username', 'cmdline']):
        if ct - proc.info['create_time'] < 5 and proc.username() == usr and proc.pid not in terminated_pids and proc.pid != os.getpid() and 'python' not in proc.info['cmdline']:
            try:
    #            print("\n killing... ", proc.info['cmdline'])
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
def calc_angles(client1_pos, client2_pos):

# Calculate the differences in x and y coordinates
    dx = client2_pos[0] - client1_pos[0]
    dy = client2_pos[1] - client1_pos[1]

# Calculate the angle from point 1 to point 2
    theta1 = math.degrees(math.atan2(dy, dx))
    if theta1 < 0:
        theta1 += 360

# Calculate the angle from point 2 to point 1
    theta2 = math.degrees(math.atan2(-dy, -dx))
    if theta2 < 0:
        theta2 += 360


    # Print the results
    print("Angle from point 1 to point 2:", theta1)
    print("Angle from point 2 to point 1:", theta2)
    return theta1,theta2
frame_num = '1'
Frame_time = 0
## initialization ##
Cluster_dict = {}
Next_Cluster_dict = {}
Next_Velocity_dict = {}
## KalmanFilter for Client 1
KalmanMeasurements = np.zeros((4,1))
KalmanMeasurements[0] = -1
KalmanMeasurements[1] = 1
KalmanP = np.zeros((4,4,1))
Innovation = np.zeros((2,1))
KalmanF = np.zeros((4,2,1))
ConditionalX = np.zeros((4,1))
ConditionalP=np.identity(4)

## KalmanFilter for Client 2 

KalmanMeasurements2 = np.zeros((4,1))
KalmanMeasurements2[0] = 0.5
KalmanMeasurements2[1] = 2
KalmanP2 = np.zeros((4,4,1))
Innovation2 = np.zeros((2,1))
KalmanF2 = np.zeros((4,2,1))
ConditionalX2 = np.zeros((4,1))
ConditionalP2 = np.identity(4)

parser = argparse.ArgumentParser(description='This is a script that performs client localization and beam angle calculation based on radar and IMU data')
parser.add_argument('-time', type=int, default = 3, help='time in seconds to let radar run for,default = 3', required=False)
args = parser.parse_args()

cwd = os.getcwd() ##
print(cwd) ## make sure we're in the right place modify accordingly
##client create

fig, ax = plt.subplots()
scatter = ax.scatter([], [])
client1_coords = [[0,0],[0,0]]
client2_coords = [[0,0],[0,0]]
ax.set_xlim([-2, 2])
ax.set_ylim([0, 4.5])

f = 0
client_ips = []

Start = False
Track = False
Distance = False


#start client threads
for i in range(2):
    client_address = ("", PORT + i + 1)
    Thread(target = handle_client, args = (client_address, clients, clients_lock, client_ips), daemon=True).start()


while True: ## get 5 global frames ## change to True later
    run_radar_collection()
    data_df = pd.read_csv('test.csv')
    nunique_frames = data_df.iloc[:,1].nunique()
    print(nunique_frames)
    if nunique_frames < 2:
        # if we didn't get enough frames rerun data collection
        print('less than 2 frames')
        continue
    #here we can edit the csv file.
    #if not both connected then continue
    with clients_lock:
        if len(client_ips) < 2:
            print("not connected")
            continue
    with clients_lock:
        #ensure clients always the same
        #print id(clients)
        #print('client_ips: ', client_ips)
        #print('clientes_dict', clients)
        client1 = clients[client_ips[0]]
        client2 = clients[client_ips[1]]
    print(f'### client1 ###\n{client1}\n### client2 ###\n{client2}')
    #check_connections() 
    if Start == False:
        Start_time = time.time()
        Frame_time = 1
    else:
        Stop_time = time.time()
        Frame_time = Stop_time - Start_time
        Start_time = time.time()
    
    
    #check what the name of this column is#
    num_frames = data_df.iloc[:,1].nunique()
    #print("DEBUG: Number of unique frames: ", num_frames)
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
    Cluster_dict, Next_Cluster_dict, Next_Velocity_dict = Frame.updatecluster(CorePoints,Next_Cluster_dict,n_clusters_, Frame_time, Distance, client1, client2)

    # populate client imuFrame var with iframe from server

    R.trackOrientation6D(Frame_time,client1)
    R.trackOrientation6D(Frame_time,client2)
    R.getVelocity(Frame_time,client1,client2)
    # Update Orientation
    R.quaternion_to_euler(client1)
    R.quaternion_to_euler(client2)
    ## Identify router and return label of that router, runs every global frame
    if f == 1:
        Distance = True

    if Start == True:

        if Track == True:
            client1_id, client2_id = Frame.findrouter(client1, client2, Next_Velocity_dict, Next_Cluster_dict, Distance)
            #client2_id = Frame.findrouter(client2, Next_Velocity_dict)
            print(f'\nclientid1:{client1_id}')
            print(f'\nclientid2:{client2_id}')
        
        try: 
            KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP = Frame.kalmanFilter\
                    (client1,Next_Cluster_dict,KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP)
            
            KalmanMeasurements2,KalmanP2,Innovation2,KalmanF2,ConditionalX2,ConditionalP2 = Frame.kalmanFilter\
                    (client2,Next_Cluster_dict,KalmanMeasurements2,KalmanP2,Innovation2,KalmanF2,ConditionalX2,ConditionalP2)
        except:
            print(f'except condition met')
            client1_id, client2_id = Frame.findrouter(client1, client2, Next_Velocity_dict, Next_Cluster_dict, Distance)

        ## Get Beamforming angle
        Theta = Frame.getEstimate (client1,client2)

        canBeamForm, Beamangle1 = R.beamform_angle(Theta,client1)
        print(f'beama: {Beamangle1}')
        Beamangl1 = 0
        #why is there only one beam angle?
        Beamangle2 = 180
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            with clients_lock:
                s.sendto(str(Beamangle1).encode(), client1.id)
                s.sendto(str(Beamangle2).encode(), client2.id)
            print("#### Sending to {}:{} ####".format(client_ips[0], Beamangle1))
            print("#### Sending to {}:{} ####".format(client_ips[1], Beamangle2))
    
        client1_pos = [KalmanMeasurements[0][0], KalmanMeasurements[1][0]]
        client2_pos = [KalmanMeasurements2[0][0], KalmanMeasurements2[1][0]]
        Beamangle1, Beamangle2 = calc_angles(client1_pos, client2_pos)
        Beamangles = {Beamangle1: None, Beamangle2: None}
        with open('angles_client1.txt', 'w') as f:
            f.write('BA: {Beamangle1}\n')       
        with open('angles_client2.txt', 'w') as f:
            f.write('BA: {Beamangle2}\n')   
        for beamangle in Beamangles.keys():
            if beamangle < -90 or beamangle > 90:
                if beamangle > 30:
                    print('sector C')
                    beamangle-= 60
                elif beamangle < -30:
                    print('sector A')
                    beamangle += 60
                else:
                    print('sector B')
            else:
                print('cannot actually beamform')
                Beamangles[beamangle] = tx_dict[beamangle]
        
        tx_sector_1 = Beamangles[Beamangle1]
        tx_sector_2 = Beamangles[Beamangle2]
        
        with open('angles_client1.txt', 'w') as f:
            f.write('position: {client1_pos} angle:{Beamangle1} tx_sector ={tx_sector_1}\n\n')
        with open('angles_client2.txt', 'w') as f:
            f.write('position: {client2_pos} angle:{Beamangle2} tx_sector ={tx_sector_2}\n\n')
        client1_coords.append(client1_pos)
        client2_coords.append(client2_pos)
        #client1_last_two_coords = np.vstack(client1_coords[-2:])
        #client2_last_two_coords = np.vstack(client2_coords[-2:])
        #all_coords = np.concatenate((client1_last_two_coords, client2_last_two_coords), axis = 0)
        x1, y1 = zip(*Client1_coords)
        x2, y2 = zip(*Client2_coords)
        plt.scatter(x1, y1, s =30, color='blue', label='Client1')
        plt.scatter(x2, y2, s = 30, color='red', label='Client2')
        
        print(f'\ncoords{all_coords}')
        colors = ['lightskyblue','b', 'lightcoral', 'r']
        x1, y1 = client1_pos
        x2, y2 = client2_pos
        width = 0.5
        height = 0.1
         # Adding an oval for client 1
         # Get the coordinates of the most recent positions of client1 and client2
         # Draw a dashed red line connecting client1 and client2

        ellipse1 = Ellipse(xy=(x1+0.075, y1), width=width, height=height, angle= Beamangle1, facecolor="yellow")
        #if canBeamForm:
        #ax.add_patch(ellipse1)
        ellipse2 = Ellipse(xy=(x2-0.075, y2), width=width, height=height, angle= Beamangle2, facecolor="yellow")
        #if canBeamForm:
        #ax.add_patch(ellipse2)
        #line = plt.plot([x1, x2], [y1, y2], color='r', linestyle='--', linewidth=0.25)
        #ground truths
        c1_gt_x1,c1_gt_y1 = 0.5, 2
        c1_gt_x2,c1_gt_y2 = 0.5, 1.5
        c1_gt_x3,c1_gt_y3 = 1, 1.5
        c1_gt_x4,c1_gt_y4 = 1, 2
        c1_gt_x5,c1_gt_y5 = 1, 2.5
        c1_gt_x6,c1_gt_y6 = 1.5, 2.5
        c1_gt_x9,c1_gt_y9 = 1.5, 2.0

        c2_gt_x1, c2_gt_y1 = -1,1
        c2_gt_x2, c2_gt_y2 = -1, 1.5
        c2_gt_x3, c2_gt_y3 = -1, 2
        c2_gt_x4, c2_gt_y4 = -0.5, 2
        c2_gt_x5, c2_gt_y5 = -0.5, 2.5
        c2_gt_x6, c2_gt_y6 = -1.0, 2.5
        c2_gt_x7, c2_gt_y7 = -1.0, 2.0        

        c1_gt_line1 = plt.plot([c1_gt_x1, c1_gt_x2], [c1_gt_y1, c1_gt_y2], color='g', linewidth=0.5)
        c1_gt_line2 = plt.plot([c1_gt_x2, c1_gt_x3], [c1_gt_y2, c1_gt_y3], color='g', linewidth=0.5)
        c1_gt_line3 = plt.plot([c1_gt_x3, c1_gt_x4], [c1_gt_y3, c1_gt_y4], color='g', linewidth=0.5)
        c1_gt_line4 = plt.plot([c1_gt_x4, c1_gt_x5], [c1_gt_y4, c1_gt_y5], color='g', linewidth=0.5)
        c1_gt_line5 = plt.plot([c1_gt_x5, c1_gt_x6], [c1_gt_y5, c1_gt_y6], color='g', linewidth=0.5)
        c1_gt_line6 = plt.plot([c1_gt_x6, c1_gt_x9], [c1_gt_y6, c1_gt_y9], color='g', linewidth=0.5)
        
        c2_gt_line1 = plt.plot([c2_gt_x1, c2_gt_x2], [c2_gt_y1, c2_gt_y2], color='g', linewidth=0.5)
        c2_gt_line2 = plt.plot([c2_gt_x2, c2_gt_x3], [c2_gt_y2, c2_gt_y3], color='g', linewidth=0.5)
        c2_gt_line3 = plt.plot([c2_gt_x3, c2_gt_x4], [c2_gt_y3, c2_gt_y4], color='g', linewidth=0.5)
        c2_gt_line4 = plt.plot([c2_gt_x4, c2_gt_x5], [c2_gt_y4, c2_gt_y5], color='g', linewidth=0.5)
        c2_gt_line5 = plt.plot([c2_gt_x5, c2_gt_x6], [c2_gt_y5, c2_gt_y6], color='g', linewidth=0.5)
        c2_gt_line6 = plt.plot([c2_gt_x6, c2_gt_x7], [c2_gt_y6, c2_gt_y7], color='g', linewidth=0.5)
        #scatter.set_offsets(all_coords)
        #scatter.set_color(colors)
        print(f'\nclient1 coords:{client1_coords}\nclient2 coords:{client2_coords}')
        print(f'\nlast_two1:{client1_last_two_coords}')

        plt.title(f'frame number: {f}')
        plt.legend()
        plt.draw()
        plt.savefig(f'plt_num_{f}_.png')
        plt.pause(0.1)
        l = line.pop(0)
        l.remove()
        ellipse1.remove()
        ellipse2.remove()
    Prev_Frame = Frame
    
    frame_num = str(int(frame_num)+1)

    Start = True

    if f == 0:
        Track = True
    if f == 1:
        Track = False 
    f += 1  
