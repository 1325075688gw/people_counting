import sys
import os

import copy

import time
import math

from single_cluster.src.cluster import Cluster
from single_cluster.src.points_filter import Points_Filter
from single_cluster.common import frame_data_queue,cluster_show_queue,point_cloud_show_queue



from queue import Queue
location_queue = Queue()



def test_point_filter():
    # print("thread2 start")
    #tracker = Multi_Kalman_Tracker(0.5, 30, 300, 30, 0.5, -4, 4, 8)
    p_filter = Points_Filter(z_min=-1, z_max=2.5, del_doppler=0, snr_limit=0)
    cl = Cluster(eps = 0.25,minpts = 5,type='2D',min_cluster_count = 20, cluster_snr_limit=120)
    #cl = Cluster(eps = 0.25,minpts = 5,type='2D',min_cluster_count = 0, cluster_snr_limit=0)
    all_height = []
    save_data = {}
    dist_list = []
    snr_list = []
    point_num = []
    list_width_dict = []  #宽度和距离列表，来找他们之间的关系
    cluster_count = {}
    time_count = 0
    for i in range(26):
        cluster_count[i]=0
    #print("队列长度？：",len(frame_data_queue))
    while not frame_data_queue.empty():
        time_begin = time.time()*1000
        frame_data = frame_data_queue.get()
        fd = copy.deepcopy(frame_data)
        point_cloud_show_queue.put(fd)
        #point_cloud_show_queue.put(frame_data)
        p_filter.run_filter(frame_data)

        #print("location",get_locations)
        cl.do_clsuter(frame_data)
        # cl.show_cluster_dopper()
        #cl.show_snr()
        frame_cluster_dict = copy.deepcopy(cl.frame_cluster_dict)
        out_list = []
        #people_height_list = Height.get_people_height_list(cl)
        people_height_list = cl.get_height_list()
        center_point_list = cl.get_cluster_center_point_list()
        sum_snr_list = cl.get_sum_snr_list()
        #print(center_point_list)

        for height, center_point in zip(people_height_list, center_point_list):
            tem_data = [center_point[0], center_point[1], height]
            dist = math.sqrt(center_point[0]*center_point[0]+center_point[1]*center_point[1])
            dist_list.append(dist)
            out_list.append(tem_data)
        for snr in sum_snr_list:
            snr_list.append(snr)
        save_data[cl.frame_cluster_dict['frame_num']] = out_list
        cluster_show_queue.put(frame_cluster_dict)

        locations = {}
        for i in range(len(center_point_list)):
            locations[i] = center_point_list[i]
        location_queue.put(locations)

        frame_num = frame_data["frame_num"]
        for cluster in cl.frame_cluster_dict['cluster']:
            list_width_dict.append([cluster['width'],cluster['dist']])

    return save_data

def run_cluser(json_load):
    for i in json_load:
        frame_data_queue.put(json_load[i])
    return test_point_filter()