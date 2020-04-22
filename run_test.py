import sys
sys.path.append("./src")
sys.path.append("./ui")
import threading
from queue import Queue
import copy
import matplotlib.pyplot as plt
import numpy as np
import time
import json

from cluster import Cluster
from points_filter import Points_Filter
from commo import frame_data_queue
from read_file_data import read_data
import show
from height import Height

cluster_show_queue = Queue()

def save_result(save_data):
	json_str = json.dumps(save_data)
	with open('./data/output/格力/在雷达照射方向上来回走动/data_4_22_1_1.9_在雷达照射方向上来回走动/cart_transfer_data.json', 'w') as json_file:
		json_file.write(json_str)

def test_point_filter():
	print("thread2 start")
	p_filter = Points_Filter(z_min = 0, z_max = 2.5, del_dopper = 0)
	cl = Cluster(eps = 0.25,minpts = 5,type='2D',min_cluster_count = 30)
	all_height = []
	save_data = {}
	dist = []
	point_num = []
	#print("队列长度？：",len(frame_data_queue))
	while not frame_data_queue.empty():
		print("here?")
		frame_data = frame_data_queue.get()
		p_filter.run_filter(frame_data)
		cl.do_clsuter(frame_data)
		cl.show_cluster_dopper()
		frame_cluster_dict = copy.deepcopy(cl.frame_cluster_dict)
		out_list = []
		#people_height_list = Height.get_people_height_list(cl)
		people_height_list = cl.get_height_list()
		#print(people_height_list)
		center_point_list = cl.get_cluster_center_point_list()
		for height,center_point in zip(people_height_list,center_point_list):
			tem_data = [center_point[0],center_point[1],height]
			out_list.append(tem_data)
		save_data[cl.frame_cluster_dict['frame_num']] = out_list
		#print("帧号",frame_cluster_dict['frame_num'])
		
		#for cluster in frame_cluster_dict['cluster']:
		#	print("均值",cluster['center_point'][2])
		#	print("方差",cluster['var_z'])
		cluster_show_queue.put(frame_cluster_dict)
		#print(np.mean(all_height))
		#print(np.var(all_height))
		#print(cl.get_cluster_center_point_list())
		#print(cl.get_height_list())
	save_result(save_data)
	print("eend")
	'''
	plt.figure()
	#在当前绘图对象进行绘图（两个参数是x,y轴的数据）
	plt.plot(dist,point_num,'.')
	plt.pause(60)
	'''


if __name__ == "__main__":
	t1 = threading.Thread(target=read_data)
	t2 = threading.Thread(target=test_point_filter)
	t1.start()
	time.sleep(2)
	t2.start()

	fig = plt.figure(figsize=(10,10))
	while 1:
		print("draw start")
		plt.clf()
		cluster_show = cluster_show_queue.get()
		print(cluster_show)
		show.show_for_cluster(cluster_show,fig)
		plt.title("第%d帧"%cluster_show['frame_num'])
		plt.pause(0.04)
		#print(len(cluster_show['cluster']))
		#if cluster_show['frame_num'] >865:
		#	plt.pause(15)

		


