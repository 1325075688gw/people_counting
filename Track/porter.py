from Track.tracker_door import Tracker
from Track.visual_all import run_visual

from Origin_Point_Cloud import utils
from Origin_Point_Cloud import common

import json
import multiprocessing
import numpy as np
import time

def run_track():
    data=read_data()
    # multiprocessing.Process(target=run_visual,args=(common.xmin,common.xmax,common.ymax,common.detection_range,common.loc_pos,)).start()
    analyze_data(data)
    run_visual(common.xmin,common.xmax,common.ymax,common.detection_range,common.loc_pos)


def read_data():
    filepath='data/cart_data.json'
    file=open(filepath)
    data=json.load(file)
    return data

def analyze_data(data):
    tracker=Tracker(common.door_max_accept_distance, common.longest_pause_frames, common.door_in_frames,
                    common.door_out_frames, common.door_in_rate)
    for frame in data:
        frame_data=data[frame]
        locations=dict()
        heights=dict()

        for radar in frame_data:
            np_frame_data=np.array(frame_data[radar])
            if len(np_frame_data)!=0:
                locations[radar]=np_frame_data[:,:2]
                heights[radar]=np_frame_data[:,2]
        clusters_center, people_height_list = utils.mix_radar_clusters(locations, heights)

        # common.loc_pos.put([locations,clusters_center])

        # np_frame_data = np.array(frame_data['0'])
        # if len(np_frame_data)!=0:
        #     clusters_center, people_height_list=np_frame_data[:,:2],np_frame_data[:,2]
        # else:
        #     clusters_center, people_height_list =[],[]

        tracker.nextFrame(clusters_center, people_height_list)

        locations = tracker.get_each_person_location()
        postures = tracker.get_each_person_posture()
        heights = tracker.get_each_person_height()
        origin_clusters = tracker.get_origin_clusters()

        common.loc_pos.put([locations, postures, tracker.get_cluster_num(),frame,origin_clusters])

        time.sleep(0.05)

if __name__=='__main__':
    run_track()