
import collections
def findrouter(self,client1, client2,labels):
    counter = collections.Counter(labels)
    count = counter.most_common(2)
    client1.ClusterID = count[0][0]
    client2.ClusterID = count[1][0]
    
    return
