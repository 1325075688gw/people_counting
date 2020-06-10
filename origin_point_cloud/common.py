# 作者     ：origin_point_cloud
# 创建日期 ：2020-04-09  上午 0:42
# 文件名   ：common.py

from multiprocessing import Queue
import math

#public
queue_for_cluster_transfer = Queue()
queue_for_show_transfer = Queue()
cluster_show_queue = Queue()
frame_data_queue=Queue()
loc_pos = Queue()
point_cloud_show_queue = Queue()

#cluster
divide_line = 6  # 2个雷达的分界线位置
radar_2_x = 0  # 雷达2在雷达1下的x坐标
radar_2_y = 12  # 雷达2在雷达1下的y坐标

#origin point cloud
stop_flag=False

#track
detection_range=7.5
xmin=-3
xmax=3
ymin=0
ymax=12

max_accept_distance=0.4
in_rate=0.5
in_frames=10
out_frames=10
delay_frames=10
longest_pause_frames=200
MAX_SAVE_FRAMES=delay_frames*2
doFilter=False
arg_smooth=0.1
frame_rate=0.5

evm_index=[0,1]
ports=[['COM3','COM4'],
       ['COM35','COM36'],
       ['COM21','COM22']]
relative_poses=[[0,0],
                [1.5,12],
                [-2,6]]
directions=[[0,1],
            [0,-1],
            [1,0]]
xdirections=[[1,0],
             [-1,0],
             [-1,0]]
tilts=[12,12,0]
heights=[2.2,2.2,2.2]