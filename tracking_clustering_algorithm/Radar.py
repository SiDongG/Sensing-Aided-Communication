import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN
from sklearn import metrics
import pandas as pd
from pyquaternion import Quaternion
from numpy.linalg import norm
import math

from IMU import iframe

class client():
    def __init__(self, IMUdata, ip_address):   
        self.id = ip_address
        self.x = -1
        self.y = -1
        self.x_velocity= -1
        self.y_velocity = -1
        self.z_velocity = -1
        self.x_orient=0
        self.y_orient=0
        self.z_orient=0
        self.quaternion=Quaternion(1,0,0,0)
        self.x_estimate = -1
        self.y_estimate = -1
        
        self.imuFrame = IMUdata
        self.imuFramePrev = iframe([-1,-1,-1,-1,-1,-1])
    
    def update_imu_data(self, imu_data):
        self.imuFramePrev = self.imuFrame
        self.imuFrame = imu_data

def getData(data_df):
    data = np.array([])
    unique_value = data_df.iloc[:,1].value_counts().index
    sorted_list = sorted(unique_value)
    frame_num = sorted_list[-2]
    print(frame_num)
    df_filtered = data_df[data_df.iloc[:,1]==frame_num]

    return df_filtered

def quaternion_mult(q,r):
    return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]

def Direction_Correction(point,q):
    r = [0]+point
    q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
    return quaternion_mult(quaternion_mult(q,r),q_conj)[1:]

def quaternion_to_euler(client):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw) in radians.
    Parameters:
        q (numpy.ndarray): A quaternion represented as a four-element numpy array [w, x, y, z].
    Returns:
        numpy.ndarray: A three-element numpy array [roll, pitch, yaw] in radians.
    """
    # Normalize the quaternion
    q = client.quaternion
    g = np.array([q[0],q[1],q[2],q[3]])
    q_norm = q / norm(g)

    # Extract the components of the quaternion
    w, x, y, z = q_norm

    # Calculate the roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Calculate the pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Calculate the yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    # return np.array([roll, pitch, yaw])

    # update client.x_orient, client.y_orient
    client.x_orient = roll
    client.y_orient = pitch





def trackOrientation(frame_time,client):
    beta = 1
    zeta = 0
    # 9-D Madgwick Filter 
    q = client.quaternion
    h = q * (Quaternion(0, client.imuFrame.mag_x, client.imuFrame.mag_y, client.imuFrame.mag_z)*q.conj())
    b = np.array([0, norm(h[1:3]), 0, h[3]])
    f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - client.imuFrame.accel_x,
            2*(q[0]*q[1] + q[2]*q[3]) - client.imuFrame.accel_y,
            2*(0.5 - q[1]**2 - q[2]**2) - client.imuFrame.accel_z,
            2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]) - client.imuFrame.mag_x,
            2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]) - client.imuFrame.mag_y,
            2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - client.imuFrame.mag_z
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
    gyroscopeQuat = Quaternion(0, client.imuFrame.gyro_x, client.imuFrame.gyro_y, client.imuFrame.gyro_z)
    stepQuat = Quaternion(step.T[0], step.T[1], step.T[2], step.T[3])
    gyroscopeQuat = gyroscopeQuat + (q.conj() * stepQuat) * 2 * frame_time * zeta * -1
    qdot = (q * gyroscopeQuat) * 0.5 - beta * step.T

    q += qdot * client.samplePeriod
    client.quaternion = Quaternion(q / norm(q))
    return

def trackOrientation6D(frame_time,client):
    zeta = 0
    beta = 1
    # 6-D Madgwick Filter
    q = client.quaternion
    f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - client.imuFrame.accel_x,
            2*(q[0]*q[1] + q[2]*q[3]) - client.imuFrame.accel_y,
            2*(0.5 - q[1]**2 - q[2]**2) - client.imuFrame.accel_z
        ])
    j = np.array([
        [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
        [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
        [0, -4*q[1], -4*q[2], 0]
    ])
    step = j.T.dot(f)
    step /= norm(step) 

    qdot = (q * Quaternion(0, client.imuFrame.gyro_x, client.imuFrame.gyro_y, client.imuFrame.gyro_z)) * 0.5 - beta * step.T

    q += qdot * frame_time
    g = np.array([q[0],q[1],q[2],q[3]])
    q = q/norm(g)
    client.quaternion = q

def getVelocity(frame_time,client1,client2):
    client1.x_velocity = 0.5*(client1.imuFrame.accel_x+client1.imuFramePrev.accel_x)*frame_time
    client1.y_velocity = 0.5*(client1.imuFrame.accel_y+client1.imuFramePrev.accel_y)*frame_time
    client1.z_velocity = 0.5*(client1.imuFrame.accel_z+client1.imuFramePrev.accel_z)*frame_time
    client2.x_velocity = 0.5*(client2.imuFrame.accel_x+client2.imuFramePrev.accel_x)*frame_time
    client2.y_velocity = 0.5*(client2.imuFrame.accel_y+client2.imuFramePrev.accel_y)*frame_time
    client2.z_velocity = 0.5*(client2.imuFrame.accel_z+client2.imuFramePrev.accel_z)*frame_time
    return 


def beamform_angle(Theta, client1):
    angle_orient = math.atan2(client1.y_orient, client1.x_orient)  # orient of the client

    ori_range_lowerbound = angle_orient - math.pi / 2
    ori_range_upperbound = angle_orient + math.pi / 2

    if Theta >= ori_range_lowerbound and Theta <= ori_range_upperbound:
        print('Can beamform')
        BF_angle = abs(Theta - angle_orient)
        angle_from_lowerbound = Theta - ori_range_lowerbound
        # beamform angle is some radian to the left or right of the center, so BF_angle in [0,pi/2]
        if angle_from_lowerbound < math.pi/2:
            print('Right side of center')
        if angle_from_lowerbound > math.pi/2:
            print('Left side of center')
            BF_angle = Theta - ori_range_lowerbound - math.pi/2
    else:
        print('Cannot beamform')
        BF_angle = 100  #  does not mean angle is 100, mean to be set as AUTO mode for router

    return BF_angle  # in radian


class rframe():
	
    def __init__(self, frame_num, data, clients, prev_frame, start_time):   
        self.frame_num = frame_num
        self.data = data #csv data, might need to be converted to pandas object
        self.clients = clients # array of clients with IMU data
        self.prev_frame = prev_frame
        self.frame_start_time = start_time

    def getCluster(self):
		### preform DBscan, eliminate noise ##
        data2 = self.data
        data2 = data2.iloc[:,3:6]
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
        Index = self.data.index[0]
        for point in range(0,len(self.data)):
            if labels[point] != -1:
                CoreNum[labels[point]] = CoreNum[labels[point]] + 1
                CoreSum[labels[point]][0] = CoreSum[labels[point]][0] + self.data.loc[point+Index].iat[3]
                CoreSum[labels[point]][1] = CoreSum[labels[point]][1] + self.data.loc[point+Index].iat[4]
                CoreSum[labels[point]][2] = CoreSum[labels[point]][2] + self.data.loc[point+Index].iat[5]

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
    
    def findrouter(self, client, Next_Velocity_dict):
        r = [client.x_velocity, client.y_velocity, client.z_velocity]
        q = client.quaternion
        New_r = Direction_Correction(r,q)
        client.x_velocity = New_r[0]
        client.y_velocity = New_r[1]
        client.z_velocity = New_r[2]
    ###find the router###
        Min = 100
        Min_id = 100
        for keys in Next_Velocity_dict:
            dis =  (Next_Velocity_dict[keys][0]-New_r[0])**2+(Next_Velocity_dict[keys][1]-New_r[1])**2+(Next_Velocity_dict[keys][2]-New_r[2])**2
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
        client.x = KalmanMeasurements[0]
        client.y = KalmanMeasurements[1]
        
        return KalmanMeasurements,KalmanP,Innovation,KalmanF,ConditionalX,ConditionalP
		# estimate x,y leveraging imu data

 
    def getEstimate(self, client1, client2):
        ## Calculates the Direction (in radian, between 0 and 360) of client2 wrt client 1
        ratio = (client2.y-client1.y)/(client2.x-client1.x)
        Theta = np.arctan(ratio)
        if client2.x-client1.x < 0:
            Theta = Theta + math.pi
        return Theta