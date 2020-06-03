# 作者     ：gw
# 创建日期 ：2020-04-09  上午 0:42
# 文件名   ：common.py

from multiprocessing import Queue
import math

queue_for_cluster_transfer = Queue()
queue_for_show_transfer = Queue()
cluster_show_queue = Queue()
frame_data_queue=Queue()
loc_pos = Queue()
loc_pos_empty=True

point_cloud_show_queue = Queue()

detection_range=7.5
xmin=-detection_range/2*math.sqrt(3)
xmax=detection_range/2*math.sqrt(3)
ymin=0
ymax=detection_range

min_accept_distance=0.6
in_rate=0.5
delay_frames=min_in_last_times=min_out_last_times=10

stop_flag=False