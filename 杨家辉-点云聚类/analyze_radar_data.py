import sys
import matplotlib.pyplot as plt
import math
import numpy as np

import common
from common import cluster_show_queue
import show

sys.path.append(r"../龚伟-点云检测")
sys.path.append(r"../郭泽中-跟踪、姿态识别")

from Kalman import Multi_Kalman_Tracker
from cluster import Cluster
from points_filter import Points_Filter


def cluster_points():
	"""
	对点云进行过滤，聚类
	:return: None
	"""
	tracker=Multi_Kalman_Tracker(0.5, 30, 300, 30, 0.5, -4, 4, 8)
	p_filter = Points_Filter(z_min=0, z_max=2.5, del_doppler=0)
	cl = Cluster(eps=0.25, minpts=5, type='2D', min_cluster_count=30)

	while 1:
		frame_data = common.queue_for_cluster_transfer.get()
		p_filter.run_filter(frame_data)
		cl.do_clsuter(frame_data)
		clusters_center = cl.get_cluster_center_point_list()
		people_height_list = cl.get_height_list()
		frame_cluster_dict = copy.deepcopy(cl.frame_cluster_dict)
		cluster_show_queue.put(frame_cluster_dict)
		frame_num = frame_data["frame_num"]
		tracker.nextFrame(clusters_center, people_height_list, frame_num)
		locations = tracker.get_each_person_location()
		postures = tracker.get_each_person_posture()
		common.queue_for_show_transfer.put([frame_num, locations, postures])


def show_track():
	"""
	可视化
	:return: None
	"""
	num = 0
	plt.ion()
	plt.figure(figsize=(8, 10))
	angle = np.arange(math.pi / 3, (math.pi * (2 / 3)), math.radians(60)/20)
	angle = angle[1:]
	x = np.cos(angle)*8
	y = np.sin(angle)*8
	x = np.insert(x,0,0)
	y = np.insert(y,0,0)
	x = np.insert(x,len(x),0)
	y = np.insert(y,len(y),0)
	while 1:
		frame_num, positions, postures = common.queue_for_show_transfer.get()
		num += 1
		if num < 4:
			continue
		else:
			num = 0
			show.track(frame_num, positions, postures, x, y)
			plt.pause(0.0001)
			plt.clf()
