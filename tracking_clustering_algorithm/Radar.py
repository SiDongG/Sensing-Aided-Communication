#import IMU

import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import csv
import os
import sys
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN
from sklearn import metrics
import pandas as pd

class client():
	def __init__(self, IMU_data):
		self.x = -1
		self.y = -1
		self.x_velocity= -1
		self.y_velocity = -1
		self.x_estimate = -1
		self.y_estimate = -1
		self.imuFrame = IMU_data

def getData(frame_num, currentFileCSV):
    data = np.array([])
    with open(currentFileCSV) as file:
        reader_obj = csv.reader(file)
        for row in reader_obj:
            if row[1] == frame_num:    
                data = np.append(data,row,axis = 0)
            elif row[1] > frame_num:
                Next_frame_num = row[1]
                break
                
    data = np.reshape(data,(int(len(data)/8),8))
    data = data.astype(float)

    return data, Next_frame_num

class rframe():
	
    def __init__(self, frame_num, data, clients, prev_frame):   
        self.frame_num = frame_num
        self.data = data #csv data, might need to be converted to pandas object
        self.clients = clients # array of clients with IMU data
        self.prev_frame = prev_frame

    def getCluster(self):
		### preform DBscan, eliminate noise ##
        data2 = self.data[:,3:6]
        db = DBSCAN(eps=0.3, min_samples=25).fit(data2)
        labels = db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)  # Number of Clusters, Label 0-N
        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        return labels,n_clusters_

    def getcorePoint(self,n_clusters_,labels):
        CorePoints = np.zeros((n_clusters_,3))
        CoreNum = np.zeros((n_clusters_))
        CoreSum = np.zeros((n_clusters_,3))

        for point in range(0,len(self.data)):
            if labels[point] != -1:
                CoreNum[labels[point]] = CoreNum[labels[point]] + 1
                CoreSum[labels[point]][0] = CoreSum[labels[point]][0] + self.data[point][3]
                CoreSum[labels[point]][1] = CoreSum[labels[point]][1] + self.data[point][4]
                CoreSum[labels[point]][2] = CoreSum[labels[point]][2] + self.data[point][5]

        for corepoint_id in range(0,n_clusters_):
            CorePoints[corepoint_id] = CoreSum[corepoint_id]/CoreNum[corepoint_id]
        return CorePoints
    
    def updatecluster(self,CorePoints,Next_Cluster_dict,n_clusters_,Next_Velocity_dict):
        Cluster_dict = Next_Cluster_dict
        if Cluster_dict:
            if len(Cluster_dict) >= n_clusters_:
                for corepoint_id in range (0,n_clusters_):
                    Min_dis = 100
                    Min_key = 100
                    for keys in Cluster_dict:
                        dis = (Cluster_dict[keys][0]-CorePoints[corepoint_id][0])**2+(Cluster_dict[keys][1]-CorePoints[corepoint_id][1])**2\
                                +(Cluster_dict[keys][2]-CorePoints[corepoint_id][2])**2
                        if dis < Min_dis:
                            Min_key = keys
                            Min_dis = dis
                    Next_Cluster_dict[Min_key] = CorePoints[corepoint_id]
            elif len(Cluster_dict) < n_clusters_:
                for keys in Cluster_dict:
                    Min_dis = 100
                    Min_core_id = 100
                    for corepoint_id in range(0,n_clusters_):
                        dis = (Cluster_dict[keys][0]-CorePoints[corepoint_id][0])**2+(Cluster_dict[keys][1]-CorePoints[corepoint_id][1])**2\
                                +(Cluster_dict[keys][2]-CorePoints[corepoint_id][2])**2
                        if dis < Min_dis:
                            Min_core_id = corepoint_id
                            Min_dis = dis
                    Next_Cluster_dict[keys] = CorePoints[Min_core_id]

            for keys in Cluster_dict:
                if keys in Next_Cluster_dict:
                    Next_Velocity_dict[keys] = (Next_Cluster_dict[keys]-Cluster_dict[keys])/0.18
            
        else:
            for corepoint_id in range(0,n_clusters_):
                Cluster_dict[corepoint_id] = CorePoints[corepoint_id]

        return Cluster_dict, Next_Cluster_dict, Next_Velocity_dict
    
    def findrouter(self, client):
        """ client_1_imu_data = clients[0].imuFrame
        client_2_imu_data = clients[1].imuFrame """
        ###find the router###
        return client

    def kalmanFilter(self, client):
        client_imu_data = client.imuFrame
        x,y = 0 
        return x,y
		# estimate x,y leveraging imu data

 
    def getEstimate(self):
    #logic before kalman filter
        """ final_clients = kalmanFilter(self,client)
        return final_clients #final clients is array of clients with all member variables finalized. """