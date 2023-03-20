
import Radar as R
import matplotlib.pyplot as plt
import numpy as np

currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
frame_id = 1
frame_num = '1'

Cluster_dict = {}
Next_Cluster_dict = {}
Velocity_dict = {}
Next_Velocity_dict = {}
total_Cluster = []
total_Cluster_Kalman = []
KalmanMeasurements = np.zeros((4,1))
KalmanP = np.zeros((4,4,1))
Innovation = np.zeros((2,1))
KalmanF = np.zeros((4,2,1))
ConditionalX = np.zeros((4,1))
ConditionalP=np.identity(4)

for i in range(0,40):
    data, Next_frame = R.getData(frame_num, currentFileCSV)
    if frame_id == 1:
        Frame = R.rframe(frame_num,data,None,None)
    else:
        Frame = R.rframe(frame_num,data,None,Prev_Frame)
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


