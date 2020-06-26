import threading
import multiprocessing
import json
import time
import os

import common
import analyze_radar_data
from Track.visual_all import run_visual

def read_data():
    pathdir='data/data_6_26，ODS6m,7人，两块板子，第1次/'
    for i in range(len(os.listdir(pathdir))):
        file=open(pathdir+'cart_data'+str(i+1)+'.json')
        data=json.load(file)
        for frame in data:
            common.queue_for_cluster_transfer.put(data[frame])
            # time.sleep(0.05)

def start_read_data():
    threading.Thread(target=read_data).start()

def start_analyze_data():
    multiprocessing.Process(target=analyze_radar_data.cluster_points,args=(2,common.queue_for_cluster_transfer,common.cluster_show_queue,common.loc_pos,common.point_cloud_show_queue,)).start()

def spy_queue(queue_for_cluster_transfer,loc_pos):
    while True:
        print(queue_for_cluster_transfer.qsize(),loc_pos.qsize())
        time.sleep(0.05)

def start_spy(queue_for_cluster_transfer,loc_pos):
    multiprocessing.Process(target=spy_queue,args=(queue_for_cluster_transfer,loc_pos,)).start()

def start_visual():
    run_visual(common.xmin,common.xmax,common.ymax,common.detection_range,common.loc_pos)
    # multiprocessing.Process(target=run_visual,args=(common.xmin,common.xmax,common.ymax,common.detection_range,common.loc_pos,)).start()

if __name__=='__main__':
    # start_spy(common.queue_for_cluster_transfer,common.loc_pos)
    start_read_data()
    start_analyze_data()
    start_visual()