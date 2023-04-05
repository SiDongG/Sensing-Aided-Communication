import Radar as R
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import os
import time
import argparse
import pandas as pd


#currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
frame_num = '1'
## initialization ##
Cluster_dict = {}
Next_Cluster_dict = {}
Next_Velocity_dict = {}

KalmanMeasurements = np.zeros((4,1))
KalmanP = np.zeros((4,4,1))
Innovation = np.zeros((2,1))
KalmanF = np.zeros((4,2,1))
ConditionalX = np.zeros((4,1))
ConditionalP=np.identity(4)

parser = argparse.ArgumentParser(description='This is a script that performs client localization and beam angle calculation based on radar and IMU data')
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
    for Microframe in range(0,num_frames): #while true
        data, Next_frame = R.getData(frame_num, data_df)
        start_time = 0###keep track of global time

        ### Not sure if this segment is needed, since prev data is reflected in client object
        if Microframe == 0:
            Frame = R.rframe(frame_num,data,None,None, start_time)
        else:
            Frame = R.rframe(frame_num,data,None,Prev_Frame,start_time)
        
        ## Get number of clusters and labels of each point in clusters 
        labels,n_clusters_ = Frame.getCluster()
        ## Get Core points of each unique cluster
        CorePoints = Frame.getcorePoint(n_clusters_,labels)
        ## Update clusters 
        Cluster_dict, Next_Cluster_dict, Next_Velocity_dict = Frame.updatecluster(CorePoints,Next_Cluster_dict,n_clusters_,Next_Velocity_dict)
        ## Identify router and return label of that router, runs every global frame
        client1_id = Frame.findrouter(client1, Microframe ,Next_Velocity_dict)
        KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP = Frame.kalmanFilter\
             (client1_id,Next_Cluster_dict,KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP)
    
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
