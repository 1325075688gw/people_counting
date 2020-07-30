import threading
import multiprocessing
import json
import time
import os

from Origin_Point_Cloud import common
from Origin_Point_Cloud import analyze_radar_data
from Track.visual import run_visual
from Origin_Point_Cloud.utils import spy_queue
from Origin_Point_Cloud.visual_four import visual4plots

def read_data():
    pathdir='data/data_7_27，ODS6m,8人，一块板子，第1次/'
    for i in range(len(os.listdir(pathdir))):
        file=open(pathdir+'cart_data'+str(i+1)+'.json')
        data=json.load(file)
        for frame in data:
            common.queue_for_cluster_transfer.put(data[frame])
            # time.sleep(0.05)

def start_read_data():
    threading.Thread(target=read_data).start()

def start_analyze_data():
    multiprocessing.Process(target=analyze_radar_data.cluster_points,args=(3,common.queue_for_cluster_transfer,common.cluster_show_queue,common.loc_pos,common.point_cloud_show_queue,)).start()

def start_spy(queue_for_cluster_transfer,loc_pos,cluster_show_queue):
    multiprocessing.Process(target=spy_queue,args=(queue_for_cluster_transfer,loc_pos,cluster_show_queue,)).start()

def start_visual():

    # run_visual(common.xmin,common.xmax,common.ymax,common.detection_range,common.loc_pos)
    # multiprocessing.Process(target=run_visual,args=(common.xmin,common.xmax,common.ymax,common.detection_range,common.loc_pos,)).start()
    multiprocessing.Process(target=visual4plots,args=(common.loc_pos,common.point_cloud_show_queue,common.cluster_show_queue,common.xmin,common.xmax,common.ymin,common.ymax,common.detection_range,)).start()

if __name__=='__main__':
    config = common.config
    common.zmax = multiprocessing.Value('d', float(config.get('radar_params', 'zmax'))).value
    common.xmin = multiprocessing.Value('d', -5).value - 0.5
    common.xmax = multiprocessing.Value('d', 5).value + 0.5
    common.ymax = multiprocessing.Value('d', 6).value + 0.5
    common.ymin = multiprocessing.Value('d', -0.5).value
    # start_spy(common.queue_for_cluster_transfer,common.loc_pos,common.cluster_show_queue)
    start_read_data()
    start_analyze_data()
    start_visual()