#!/bin/bash
source /home/marga_share/RETINA_4SN_RVIZ_DevMode_orig/rviz/4sn_viewer/catkin_ws/devel/setup.bash
/opt/ros/noetic/bin/roslaunch /home/marga_share/RETINA_4SN_RVIZ_DevMode_orig/rviz/4sn_viewer/catkin_ws/src/retina4sn_viewer/share/srs_4sn.launch output_file:=/home/marga_share/group6/AWN/tracking_clustering_algorithm/$1
#/opt/ros/noetic/bin/rosclean purge -y
