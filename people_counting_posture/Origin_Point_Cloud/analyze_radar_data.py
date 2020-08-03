import sys
import copy

sys.path.append('../')

from Cluster import cluster_common
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

    try:
        tracker=Tracker(max_accept_distance, longest_pause_frames, in_frames, out_frames, in_rate)
    except:
        print('跟踪器初始化出错')

    try:
        cl = Cluster(eps=0.25, minpts=5, type='2D', min_cluster_count=cluster_common.min_cluster_count,
                     cluster_snr_limit=cluster_common.cluster_snr_limit)
    except:
        print('聚类初始化出错')

    start = False
    false_blocks = dict()
    frame_num = 0
    people_num = 9

    all_postures=0
    lying=0

    while 1:
        frame_data = queue_for_cluster_transfer.get()

        try:
            cl.do_cluster(frame_data)
        except:
            print('聚类出错')

        frame_cluster_result = copy.deepcopy(cl.frame_cluster_result)

        if show_flag==3:
            cluster_show_queue.put(frame_cluster_result)
            cl.put_points_show(point_cloud_show_queue)

        clusters_center = cl.get_cluster_center_point_list()
        people_height_list = cl.get_height_list()

        try:
            clusters_center,people_height_list=mix_radar_clusters(clusters_center,people_height_list)
        except Exception as e:
            print('匹配出错',e,clusters_center,people_height_list)

        frame_num +=1

        try:
            tracker.nextFrame(clusters_center, people_height_list)
            locations = tracker.get_each_person_location()
            postures = tracker.get_each_person_posture()
            heights=tracker.get_each_person_height()
            origin_clusters=tracker.get_origin_clusters()
        except Exception as e:
            print('跟踪出错',e,clusters_center,people_height_list)


        # if not start and len(locations)==people_num:
        #     start=True
        # if start and len(locations)!=people_num:
        #     delta=abs(people_num-len(locations))
        #     if delta in false_blocks:
        #         false_blocks[delta][-1]+=1
        #     else:
        #         false_blocks[delta]=[1]
        # if start and len(locations)==people_num:
        #     for delta in false_blocks:
        #         if false_blocks[delta][-1]!=0:
        #             false_blocks[delta].append(0)
        # print(start,frame_num,{delta:sum(false_blocks[delta]) for delta in false_blocks},sum([sum(false_blocks[delta]) for delta in false_blocks]))

        # if frame_num>150:
        #     if len(postures)>0:
        #         all_postures+=1
        #         if (1 in list(postures.values()) or 4 in list(postures.values())) and len(postures)==2:
        #             lying+=1
        #     if all_postures!=0:
        #         print(lying/all_postures)

        loc_pos.put([locations, postures, tracker.get_cluster_num(),frame_num,origin_clusters])
        # loc_pos.put([locations, heights, tracker.get_cluster_num(),frame_num,origin_clusters])