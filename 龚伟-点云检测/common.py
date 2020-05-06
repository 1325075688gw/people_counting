# 作者     ：gw
# 创建日期 ：2020-04-09  上午 0:42
# 文件名   ：common.py

from queue import Queue
import math

queue_for_cluster_transfer = Queue()
queue_for_show_transfer = Queue()
cluster_show_queue = Queue()
loc_pos = Queue()

detection_range=8
xmin=-detection_range/2*math.sqrt(3)
xmax=detection_range/2*math.sqrt(3)
ymax=detection_range

M=30
G=1
min_in_last_times=30
min_out_last_times=60
rate=0.5

stop_flag=False