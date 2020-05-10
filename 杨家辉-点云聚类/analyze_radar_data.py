import sys
import matplotlib.pyplot as plt
import math
import numpy as np
import copy


import show

sys.path.append(r"../龚伟-点云检测")
sys.path.append(r"../郭泽中-跟踪、姿态识别")
from common import *


from Kalman import Multi_Kalman_Tracker
from cluster import Cluster
from points_filter import Points_Filter


def cluster_points():
	"""
	对点云进行过滤，聚类
	:return: None
	"""
	tracker = Multi_Kalman_Tracker(G, min_in_last_times, min_out_last_times, M, rate, xmin, xmax, ymax)
	p_filter = Points_Filter(z_min=-0.5, z_max=2.5, del_doppler=0, snr_limit=0)
	cl = Cluster(eps=0.25, minpts=5, type='2D', min_cluster_count=15, cluster_snr_limit=100)

	gzz_heights = []
	yjh_heights = []
	while 1:
		frame_data = queue_for_cluster_transfer.get()
		fd = copy.deepcopy(frame_data)
		point_cloud_show_queue.put(fd)
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
		heights = tracker.get_each_person_height()
		temp = 0
		for i in heights:
			temp += heights[i]
		if len(heights) == 0:
			continue
		temp = temp/len(heights)
		gzz_heights.append(temp)
		if len(people_height_list) > 1:
			yjh_hei = np.mean(people_height_list)
		else:
			if len(people_height_list) == 0:
				continue
			yjh_hei = people_height_list[0]
		yjh_heights.append(yjh_hei)
		print("yjh_heights:{0}  gzz_heights:{1}".format(yjh_hei, temp))
		print("yjh_mean_heights:{0}  gzz_mean_heights:{1}".format(np.mean(yjh_heights), np.mean(gzz_heights)))
		#common.queue_for_show_transfer.put([frame_num, locations, postures])
		# loc_pos.put([locations, postures, tracker.get_frame()])


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
