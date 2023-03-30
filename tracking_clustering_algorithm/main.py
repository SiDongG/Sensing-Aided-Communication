
import Radar as R
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import os
import time
import argparse


#currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
frame_id = 1
frame_num = '1'
## initialization ##
Cluster_dict = {}
Next_Cluster_dict = {}
Velocity_dict = {}
Next_Velocity_dict = {}
total_Cluster = [] #array of all clusters for all time might not need
total_Cluster_Kalman = []
KalmanMeasurements = np.zeros((4,1))
KalmanP = np.zeros((4,4,1))
Innovation = np.zeros((2,1))
KalmanF = np.zeros((4,2,1))
ConditionalX = np.zeros((4,1))
ConditionalP=np.identity(4)

parser = argpars.ArgumentParser(description='This is a script that performs client localization and beam angle calculation based on radar and IMU data')
parser.add_argument('time', type=int, help='time in seconds to let radar run for', required = True)
args = parser.parse_args()

cwd = os.getcwd() ##
print(cwd) ## make sure we're in the right place modify accordingly

x = 0
while(x < 2): ## get 2 global frames ## change to True later
    collect_radar_data = './run.sh data.csv'
    process = subprocess.Popen(collect_radar_data, cwd=cwd)
    time.sleep(args.time)
    process.terminate()
    
    data_df = pd.read_csv('data.csv')
    #check what the name of this column is#
    num_frames = data_df.iloc[:,1].nunique()
    print("DEBUG: Number of unique frames: ", num_frames)
    ##TODO migrate these calls the global frame function##
    for i in range(0,num_frames): #while true
        data, Next_frame = R.getData(frame_num, data_df)
        start_time = 0###keep track of global time
    
        if frame_id == 1:
            Frame = R.rframe(frame_num,data,None,None, start_time)
        else:
            Frame = R.rframe(frame_num,data,None,Prev_Frame,start_time)
        labels,n_clusters_ = Frame.getCluster()
        CorePoints = Frame.getcorePoint(n_clusters_,labels)
        Cluster_dict, Next_Cluster_dict, Next_Velocity_dict = Frame.updatecluster(CorePoints,Next_Cluster_dict,n_clusters_,Next_Velocity_dict)
        client1_id,client2_id = Frame.findrouter(Next_Velocity_dict)
        KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP = Frame.kalmanFilter\
             (client1_id,Next_Cluster_dict,KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP)
    
        Prev_Frame = Frame
        frame_num = Next_frame
        frame_id = frame_id + 1
        Velocity_dict = Next_Velocity_dict
    
        NoisyMeasurements = Cluster_dict[0][0:2]
        total_Cluster.append(NoisyMeasurements)
        total_Cluster_Kalman.append(KalmanMeasurements)
    
    
    
    
    fig = plt.figure()
    ax =  fig.add_subplot(projection='3d')
    
    for row in total_Cluster_Kalman:
        ax.scatter(row[0],row[1])
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    
    plt.show()
    x += 1        
    
    