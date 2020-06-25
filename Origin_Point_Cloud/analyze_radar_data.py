import sys
import matplotlib.pyplot as plt
import math
import numpy as np
import copy
import time

from Origin_Point_Cloud import common

from Track.tracker import Tracker
from Cluster.muti_radar_cluster import Cluster
from Origin_Point_Cloud.utils import mix_radar_clusters

def cluster_points(show_flag,queue_for_cluster_transfer,cluster_show_queue,loc_pos,point_cloud_show_queue):
    """
    对点云进行过滤，聚类
    :return: None
    """
    max_accept_distance = common.max_accept_distance
    in_frames=common.in_frames
    out_frames=common.out_frames
    in_rate = common.in_rate
    longest_pause_frames=common.longest_pause_frames


    tracker=Tracker(max_accept_distance, longest_pause_frames, in_frames, out_frames, in_rate)

    cl = Cluster(eps=0.25, minpts=5, type='2D', min_cluster_count=25, cluster_snr_limit=120,radar_num=len(common.evm_index))

    cluster_nums=[]

    while 1:
        frame_data = queue_for_cluster_transfer.get()

        cl.do_cluster(frame_data,point_cloud_show_queue)

        if show_flag==1:
            frame_cluster_result = copy.deepcopy(cl.frame_cluster_result)
            cluster_show_queue.put(frame_cluster_result)
            continue

        clusters_center = cl.get_cluster_center_point_list()
        people_height_list = cl.get_height_list()

        # door_tracker.nextFrame(clusters_center[2],people_height_list[2])
        # locations=door_tracker.get_each_person_location()
        # del clusters_center[2]
        # del people_height_list[2]

        clusters_center,people_height_list=mix_radar_clusters(clusters_center,people_height_list)

        if len(clusters_center)!=0:
            cluster_nums.append(len(clusters_center))
        print('平均聚类个数:',np.mean(cluster_nums))

        frame_num = frame_data["frame_num"]
        tracker.nextFrame(clusters_center, people_height_list)


        locations = tracker.get_each_person_location()
        postures = tracker.get_each_person_posture()
        heights=tracker.get_each_person_height()
        origin_clusters=tracker.get_origin_clusters()
        loc_pos.put([locations, postures, tracker.get_cluster_num(),frame_num,origin_clusters])

        # loc_pos.put([locations, heights, tracker.get_cluster_num(),assignment,frame_num,origin_clusters])