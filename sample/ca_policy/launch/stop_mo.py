#!/usr/bin/env python

import os

os.system('rosnode kill move_base')
os.system('rosnode kill cartographer_node cartographer_occupancy_grid_node')
os.system('rosnode kill realsense_ros_camera realsense_ros_camera_manager')
os.system('rosnode kill /object_analytics/ObjectAnalyticsMgr /object_analytics/merger /object_analytics/movidius_ncs_stream /object_analytics/segmenter /object_analytics/splitter /object_analytics/tracker')
os.system('rosnode kill moving_object')
os.system('rosnode kill ca_policy')
