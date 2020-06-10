import sys
import matplotlib.pyplot as plt
import math
import numpy as np
import copy
import time

# sys.path.append(r"../龚伟-点云检测")
# sys.path.append(r"../郭泽中-跟踪、姿态识别")

import common
from common import cluster_show_queue
import show

from Kalman import Multi_Kalman_Tracker
from cluster import Cluster
from points_filter import Points_Filter


def cluster_points(show_flag):
    """
    对点云进行过滤，聚类
    :return: None
    """
    delay_frames = common.delay_frames
    min_accept_distance = common.min_accept_distance
    min_in_last_times = common.min_in_last_times
    min_out_last_times = common.min_out_last_times
    in_rate = common.in_rate


    tracker=Multi_Kalman_Tracker(min_accept_distance, min_in_last_times, min_out_last_times, delay_frames, in_rate, common.xmin, common.xmax, common.ymax)
    p_filter = Points_Filter(z_min=-1, z_max=2.5, del_doppler=0, snr_limit=0,range_x_min=common.xmin,range_x_max=common.xmax,range_y_min=common.ymin,
                             range_y_max=common.ymax)
    cl = Cluster(eps=0.25, minpts=5, type='2D', min_cluster_count=8, cluster_snr_limit=50)

    while 1:
        frame_data = common.queue_for_cluster_transfer.get()
        p_filter.run_filter(frame_data)
        cl.do_cluster(frame_data)
        clusters_center = cl.get_cluster_center_point_list()
        people_height_list = cl.get_height_list()
        frame_cluster_result = copy.deepcopy(cl.frame_cluster_result)
        cluster_show_queue.put(frame_cluster_result)

        if show_flag==1:
            continue

        frame_num = frame_data["frame_num"]
        tracker.nextFrame(clusters_center, people_height_list, frame_num)
        locations = tracker.get_each_person_location()
        postures = tracker.get_each_person_posture()
        assignment=tracker.get_assignment()
        heights=tracker.get_each_person_height()
        common.loc_pos.put([locations, postures, tracker.get_cluster_num(),assignment,frame_num])
        # common.loc_pos.put([locations, heights, tracker.get_cluster_num(),assignment,frame_num])