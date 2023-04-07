import Radar as R
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import os
import time
import argparse
import pandas as pd
import time


#currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
frame_num = '1'
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
client1 = R.client(id = 1)
client2 = R.client(id = 2)

HOST, PORT = "192.168.88.21", 1234
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST,PORT))

while(x < 2): ## get 2 global frames ## change to True later
    collect_radar_data = './run.sh data.csv'
    process = subprocess.Popen(collect_radar_data, cwd=cwd)
    time.sleep(args.time)
    process.terminate()
    data, addr = sock.recvfrom(1024)

    imu_data = [float(x) for x in data.decode().split(',')]

    data_df = pd.read_csv('data.csv')
    #check what the name of this column is#
    num_frames = data_df.iloc[:,1].nunique()
    print("DEBUG: Number of unique frames: ", num_frames)
    ##TODO migrate these calls the global frame function##
    if frame_num == '1':
        frame_num = data_df.iloc[0,1]
    first_time = data_df.iloc[0,0]
    last_time = data_df.iloc[-1,0] #this is corrupt timestamp
    num_frames = num_frames - 1 
    #remove all points from last frame.
    data_df = data_df[data_df.iloc[:, 1] != data_df.iloc[-1, 1]]

    for Microframe in range(0,num_frames): #while true
        ## Get data function returns first time, last time in one global frame and frametime between microframe (if it is not the first micro frame in global frame)
        frametime, data, Next_frame = R.getData(frame_num, data_df)
        
        #TODO review this code.
        if Microframe == 0 and x != 0:
            Stop_time = time.time()
            Frame_time = Stop_time-Start_time+first_time-last_time
            Start_time = time.time()
        elif Microframe == 0 and x == 0:
            Start_time = time.time()
        else:
            Frame_time == frametime
        ### Not sure if this segment is needed, since prev data is reflected in client object
        if Microframe == 0:
            Frame = R.rframe(frame_num,data,None,None,Start_time)
        else:
            Frame = R.rframe(frame_num,data,None,Prev_Frame,Start_time)
        
        ## Get number of clusters and labels of each point in clusters 
        labels,n_clusters_ = Frame.getCluster()
        ## Get Core points of each unique cluster
        CorePoints = Frame.getcorePoint(n_clusters_,labels)
        ## Update clusters 
        Cluster_dict, Next_Cluster_dict, Next_Velocity_dict = Frame.updatecluster(CorePoints,Next_Cluster_dict,n_clusters_,Next_Velocity_dict)
        # update client1.imudata and client2.imudata here 
        # populate client imuFrame var with iframe from server
        R.trackOrientation6D(Frame_time,client1)
        R.trackOrientation6D(Frame_time,client2)
        R.getVelocity(Frame_time,client1)
        R.getVelocity(Frame_time,client2)
        ## Identify router and return label of that router, runs every global frame
        client1_id = Frame.findrouter(client1, Microframe, Next_Velocity_dict)
        client2_id = Frame.findrouter(client2, Microframe, Next_Velocity_dict)

        KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP = Frame.kalmanFilter\
             (client1,Next_Cluster_dict,KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP)
        
        KalmanMeasurements2,KalmanP2,Innovation2,KalmanF2,ConditionalX2,ConditionalP2 = Frame.kalmanFilter\
             (client2,Next_Cluster_dict,KalmanMeasurements2,KalmanP2,Innovation2,KalmanF2,ConditionalX2,ConditionalP2)

        ## Get Beamforming angle
        Theta = Frame.getEstimate (client1,client2)

        
        Prev_Frame = Frame
        frame_num = Next_frame
    
    x += 1  
    """     fig = plt.figure()
    ax =  fig.add_subplot(projection='3d')
    
    for row in total_Cluster_Kalman:
        ax.scatter(row[0],row[1])
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    
    plt.show() """