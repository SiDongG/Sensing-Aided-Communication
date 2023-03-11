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

pd.set_option('display.expand_frame_repr', False)
np.set_printoptions(threshold=sys.maxsize)

currentDir = os.getcwd()
currentFileCSV = '/Users/shannonholmes/Desktop/Python Programs/untitled folder/label_06b.csv'

my_data = genfromtxt(currentFileCSV, delimiter=',')
Sample = my_data[0:3100]
Frame1 = my_data[0:1800]
Frame2 = my_data[1801:3100]


fig = plt.figure()
ax =  fig.add_subplot(projection='3d')

for row in Frame1:
    ax.scatter(row[3],row[4],row[5],s = row[7]/50000)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

## DBScan
data = Frame1[:,3:6]


db = DBSCAN(eps=0.5, min_samples=10).fit(data)

labels = db.labels_


n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print("Estimated number of clusters: %d" % n_clusters_)
print("Estimated number of noise points: %d" % n_noise_)

unique_labels = set(labels)
core_samples_mask = np.zeros_like(labels, dtype=bool)
core_samples_mask[db.core_sample_indices_] = True

colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]

fig = plt.figure()
bx =  fig.add_subplot(projection='3d')


for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = labels == k

    xy = data[class_member_mask & core_samples_mask]

    bx.scatter(xy[:,0],xy[:,1],xy[:,2])
    bx.set_xlabel('X Label')
    bx.set_ylabel('Y Label')
    bx.set_zlabel('Z Label')

    xy = data[class_member_mask & ~core_samples_mask]

    bx.scatter(xy[:,0],xy[:,1],xy[:,2])
    bx.set_xlabel('X Label')
    bx.set_ylabel('Y Label')
    bx.set_zlabel('Z Label')




plt.title(f"Estimated number of clusters: {n_clusters_}")

fig = plt.figure()
cx =  fig.add_subplot(projection='3d')

for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]
    xy1 = data[core_samples_mask]

    cx.scatter(xy1[:,0],xy1[:,1],xy1[:,2])
    cx.set_xlabel('X Label')
    cx.set_ylabel('Y Label')
    cx.set_zlabel('Z Label')

plt.show()

## Core points tracking

