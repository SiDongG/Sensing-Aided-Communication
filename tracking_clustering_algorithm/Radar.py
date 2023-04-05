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
import quaternion
from numpy.linalg import norm

class client():
    def __init__(self, IMU_data):   
        self.id = 0
        self.x = -1
        self.y = -1
        self.x_velocity= -1
        self.y_velocity = -1
        self.z_velocity = -1
        self.x_orient=0
        self.y_orient=0
        self.z_orient=0
        self.quaternion=quaternion(1,0,0,0)
        self.x_estimate = -1
        self.y_estimate = -1
        self.imuFrame = IMU_data
        self.imuFramePrev = IMU_data

def getData(frame_num, data_df):
    data = np.array([])
    
    # Select rows with matching frame number
    selected_rows = data_df[data_df.iloc[:, 1] == frame_num]
    
    # Convert selected rows to numpy array
    data = np.array(selected_rows)
    
    # Get the next frame number
    next_frame_num = data_df.iloc[selected_rows.index[-1]+1, 1]
                
    data = np.reshape(data, (int(len(data)/8), 8))
    data = data.astype(float)

    return data, next_frame_num

def quaternion_mult(q,r):
    return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]

def Direction_Correction(point,q):
    r = [0]+point
    q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
    return quaternion_mult(quaternion_mult(q,r),q_conj)[1:]

def trackOrientation(frame_time,client):
    beta = 1
    zeta = 0
    # 9-D Madgwick Filter 
    q = client.quaternion
    h = q * (quaternion(0, client.IMU_data.mag_x, client.IMU_data.mag_y, client.IMU_data.mag_z)*q.conj())
    b = np.array([0, norm(h[1:3]), 0, h[3]])
    f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - client.IMU_data.accel_x,
            2*(q[0]*q[1] + q[2]*q[3]) - client.IMU_data.accel_y,
            2*(0.5 - q[1]**2 - q[2]**2) - client.IMU_data.accel_z,
            2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]) - client.IMU_data.mag_x,
            2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]) - client.IMU_data.mag_y,
            2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - client.IMU_data.mag_z
        ])
    j = np.array([
        [-2*q[2],                  2*q[3],                  -2*q[0],                  2*q[1]],
        [2*q[1],                   2*q[0],                  2*q[3],                   2*q[2]],
        [0,                        -4*q[1],                 -4*q[2],                  0],
        [-2*b[3]*q[2],             2*b[3]*q[3],             -4*b[1]*q[2]-2*b[3]*q[0], -4*b[1]*q[3]+2*b[3]*q[1]],
        [-2*b[1]*q[3]+2*b[3]*q[1], 2*b[1]*q[2]+2*b[3]*q[0], 2*b[1]*q[1]+2*b[3]*q[3],  -2*b[1]*q[0]+2*b[3]*q[2]],
        [2*b[1]*q[2],              2*b[1]*q[3]-4*b[3]*q[1], 2*b[1]*q[0]-4*b[3]*q[2],  2*b[1]*q[1]]
    ])
    step = j.T.dot(f)
    step /= norm(step)
    gyroscopeQuat = quaternion(0, client.IMU_data.gyro_x, client.IMU_data.gyro_y, client.IMU_data.gyro_z)
    stepQuat = quaternion(step.T[0], step.T[1], step.T[2], step.T[3])
    gyroscopeQuat = gyroscopeQuat + (q.conj() * stepQuat) * 2 * frame_time * zeta * -1
    qdot = (q * gyroscopeQuat) * 0.5 - beta * step.T

    q += qdot * client.samplePeriod
    client.quaternion = quaternion(q / norm(q))
    return

def trackOrientation6D(frame_time,client):
    zeta = 0
    beta = 1
    # 6-D Madgwick Filter
    q = client.quaternion
    f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - client.IMU_data.accel_x,
            2*(q[0]*q[1] + q[2]*q[3]) - client.IMU_data.accel_y,
            2*(0.5 - q[1]**2 - q[2]**2) - client.IMU_data.accel_z
        ])
    j = np.array([
        [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
        [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
        [0, -4*q[1], -4*q[2], 0]
    ])
    step = j.T.dot(f)
    step /= norm(step) 

    qdot = (q * quaternion(0, client.IMU_data.gyro_x, client.IMU_data.gyro_y, client.IMU_data.gyro_z)) * 0.5 - beta * step.T

    q += qdot * frame_time
    client.quaternion = quaternion(q / norm(q))

def getVelocity(frame_time,client1,client2):
    client1.x_velocity = 0.5*(client1.imuFrame.accel_x+client1.imuFramePrev.accel_x)*frame_time
    client1.y_velocity = 0.5*(client1.imuFrame.accel_y+client1.imuFramePrev.accel_y)*frame_time
    client1.z_velocity = 0.5*(client1.imuFrame.accel_z+client1.imuFramePrev.accel_z)*frame_time
    client2.x_velocity = 0.5*(client2.imuFrame.accel_x+client2.imuFramePrev.accel_x)*frame_time
    client2.y_velocity = 0.5*(client2.imuFrame.accel_y+client2.imuFramePrev.accel_y)*frame_time
    client2.z_velocity = 0.5*(client2.imuFrame.accel_z+client2.imuFramePrev.accel_z)*frame_time
    return 


class rframe():
	
    def __init__(self, frame_num, data, clients, prev_frame, start_time):   
        self.frame_num = frame_num
        self.data = data #csv data, might need to be converted to pandas object
        self.clients = clients # array of clients with IMU data
        self.prev_frame = prev_frame
        self.frame_start_time = start_time

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

        Cluster_dict = Next_Cluster_dict.copy()

        Next_Cluster_dict = {}
        
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
                    Next_Velocity_dict[keys] = [(Next_Cluster_dict[keys][0] - Cluster_dict[keys][0]) / 0.18,
                                            (Next_Cluster_dict[keys][1] - Cluster_dict[keys][1]) / 0.18,
                                            (Next_Cluster_dict[keys][2] - Cluster_dict[keys][2]) / 0.18]
            
        else:
            for corepoint_id in range(0,n_clusters_):
                Next_Cluster_dict[corepoint_id] = CorePoints[corepoint_id]

        return Cluster_dict, Next_Cluster_dict, Next_Velocity_dict
    
    def findrouter(self, client, MicroFrameNum,Next_Velocity_dict):
        if MicroFrameNum == 0:
            r = [client.x_velocity, client.y_velocity, client.z_velocity]
            q = client.quaternion
            New_r = quaternion_mult(q,r)
            client.x_velocity = New_r[0]
            client.y_velocity = New_r[1]
            client.z_velocity = New_r[2]
        ###find the router###
            Min = 100
            Min_id = 100
            for keys in Next_Velocity_dict:
                dis =  norm(Next_Velocity_dict[keys]-New_r)
                if dis < Min:
                    Min_id = keys
                    Min = dis
            client.id = Min_id

        return client.id

    def kalmanFilter(self,client,Next_Cluster_dict,KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP):
        client_id = client.id
        SigmaInput = 1
        SigmaNoise = 0.5
        Delta = 0.18
        F = np.array([[1,0,Delta,0],[0,1,0,Delta],[0,0,1,0],[0,0,0,1]])
        Q = SigmaInput**2 * np.array([[Delta**3/3,0,Delta**2/2,0],
        [0,Delta**3/3,0,Delta**2/2],
        [Delta**2/2,0,Delta,0],
        [0,Delta**2/2,0,Delta]])
        H = np.array([[1,0,0,0],[0,1,0,0]])
        R = SigmaNoise**2 * np.identity(2)

        NoisyMeasurements = np.zeros((2,1))
        NoisyMeasurements[0,:] = Next_Cluster_dict[client_id][0]
        NoisyMeasurements[1,:] = Next_Cluster_dict[client_id][1]

        Innovation = NoisyMeasurements-H@ConditionalX
        KalmanF = ConditionalP@np.transpose(H)@np.linalg.inv(H@ConditionalP@np.transpose(H)+R)
        KalmanMeasurements = ConditionalX+KalmanF@Innovation
        KalmanP = ConditionalP-KalmanF@H@ConditionalP
        ConditionalX=F@KalmanMeasurements
        ConditionalP=F@KalmanP@np.transpose(F)+Q

        #client_imu_data = client.imuFrame
        
        return KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP
		# estimate x,y leveraging imu data

 
    def getEstimate(self):
    #logic before kalman filter
        """ final_clients = kalmanFilter(self,client)
        return final_clients #final clients is array of clients with all member variables finalized. """