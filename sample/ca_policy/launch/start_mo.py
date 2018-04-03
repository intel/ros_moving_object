#!/usr/bin/env python

import os

os.system('roslaunch waterrobot_cartographer waterrobot_cartographer_sick_lidar_2d.launch &')

os.system('roslaunch realsense_ros_camera rs_camera.launch enable_pointcloud:=true enable_sync:=true &')

os.system('roslaunch object_analytics_launch analytics_movidius_ncs.launch input_points:=/camera/points 1>/dev/null 2>&1 &')

os.system('roslaunch moving_object moving_object.launch&')

os.system('roslaunch ca_policy ca_policy.launch&')

#os.system('roslaunch state_manager rviz.launch &')
