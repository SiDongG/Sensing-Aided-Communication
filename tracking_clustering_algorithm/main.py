
import Radar as R

currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
frame_id = 1
frame_num = '13007'

Cluster_dict = {}
Next_Cluster_dict = {}
Velocity_dict = {}
Next_Velocity_dict = {}
total_Cluster = []

for i in range(0,20):
    data, Next_frame = R.getData(frame_num, currentFileCSV)
    if frame_id == 1:
        Frame = R.rframe(frame_num,data,None,None)
    else:
        Frame = R.rframe(frame_num,data,None,Prev_Frame)
    labels,n_clusters_ = Frame.getCluster()
    CorePoints = Frame.getcorePoint(n_clusters_,labels)

    Cluster_dict, Next_Cluster_dict, Next_Velocity_dict = Frame.updatecluster(CorePoints,Next_Cluster_dict,n_clusters_,Next_Velocity_dict)

    Prev_Frame = Frame
    frame_num = Next_frame
    frame_id = frame_id + 1
    Velocity_dict = Next_Velocity_dict

    print(Cluster_dict)
    NoisyMeasurements = Cluster_dict[0][0:2]
    total_Cluster.append(NoisyMeasurements)

print(total_Cluster)