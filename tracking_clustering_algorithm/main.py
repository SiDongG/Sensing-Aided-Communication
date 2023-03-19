
import Radar as R

currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/AWN/test2.csv'
frame_id = 1
frame_num = '13007'

for i in range(0,5):
    data = R.getData(frame_num, currentFileCSV)
    if frame_id == 1:
        Frame = R.rframe(frame_num,data,None,None)
    else:
        Frame = R.rframe(frame_num,data,None,Prev_Frame)
    labels,n_clusters_ = Frame.getCluster()
    CorePoints = R.getcorePoint(n_clusters_,labels)

    Prev_Frame = Frame