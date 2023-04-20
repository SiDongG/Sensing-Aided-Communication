#!/bin/bash
source /home/marga3/work/RETINA_4SN_RVIZ_DevMode/rviz/4sn_viewer/catkin_ws/devel/setup.bash
/opt/ros/noetic/bin/roslaunch /home/marga3/work/RETINA_4SN_RVIZ_DevMode/rviz/4sn_viewer/catkin_ws/src/retina4sn_viewer/share/srs_4sn.launch output_file:=/home/marga3/group6/$1
#/opt/ros/noetic/bin/rosclean purge -y
