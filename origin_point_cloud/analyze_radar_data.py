import sys
import matplotlib.pyplot as plt
import math
import numpy as np
import copy
import time

from origin_point_cloud import common

from track.tracker import Tracker
from cluster.cluster import Cluster
from cluster.points_filter import Points_Filter


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
    p_filter = Points_Filter(z_min=-1, z_max=2.5, del_doppler=0, snr_limit=0,range_x_min=-2,range_x_max=2,range_y_min=common.ymin,
                             range_y_max=12)
    cl = Cluster(eps=0.25, minpts=5, type='2D', min_cluster_count=25, cluster_snr_limit=120)

    while 1:
        frame_data = queue_for_cluster_transfer.get()
        p_filter.run_filter(frame_data)
        cl.do_cluster(frame_data,point_cloud_show_queue)

        if show_flag==1:
            frame_cluster_result = copy.deepcopy(cl.frame_cluster_result)
            cluster_show_queue.put(frame_cluster_result)
            continue

        clusters_center = cl.get_cluster_center_point_list()
        people_height_list = cl.get_height_list()
        frame_num = frame_data["frame_num"]
        tracker.nextFrame(clusters_center, people_height_list)
        locations = tracker.get_each_person_location()
        postures = tracker.get_each_person_posture()
        heights=tracker.get_each_person_height()
        origin_clusters=tracker.get_origin_clusters()

        loc_pos.put([locations, postures, tracker.get_cluster_num(),frame_num,origin_clusters])

        # loc_pos.put([locations, heights, tracker.get_cluster_num(),assignment,frame_num,origin_clusters])