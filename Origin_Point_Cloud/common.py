# 作者     ：Origin_Point_Cloud
# 创建日期 ：2020-04-09  上午 0:42
# 文件名   ：common.py

from multiprocessing import Queue
import math
import numpy as np

#public
queue_for_cluster_transfer = Queue()
queue_for_show_transfer = Queue()
queue_for_door_cart=Queue()
cluster_show_queue = Queue()
frame_data_queue=Queue()
loc_pos = Queue()
point_cloud_show_queue = Queue()

azi_range=math.pi/3
max_accept_pair_distance=1

#Cluster
divide_line = 6  # 2个雷达的分界线位置
radar_2_x = 0  # 雷达2在雷达1下的x坐标
radar_2_y = 12  # 雷达2在雷达1下的y坐标

#origin point cloud
stop_flag=False

evm_index=[1,2]

# send_config_flag=True
send_config_flag=False

ports=[['COM30','COM29'],
       ['/dev/ttyACM1','/dev/ttyACM0'],
       ['/dev/ttyACM3','/dev/ttyACM2']]
relative_poses=np.array([[-2.4,0],
                         [0,0],
                         [-7, 5]])
directions=np.array([
    [-7,5],
    [-7,5],
    [0.5855265397410541, -0.590165373739158]
])
xdirections=np.array([
    [4,-7],
    [5,7],
    [-0.590165373739158,-0.5855265397410541]])
tilts=[10,10,10]
heights=[2.2,2.1,2.1]
configuratioin_files=['./ultimate_radar_parameters_debugging.cfg','/ultimate_radar_parameters_debugging.cfg',
                      '/ultimate_radar_parameters_debugging.cfg']

save_flag=False

#Track
detection_range=6
xmin=-7
xmax=0
ymin=0
ymax=5

max_accept_distance=0.5
in_rate=0.6
in_frames=20
out_frames=10
delay_frames=10
longest_pause_frames=200
MAX_SAVE_FRAMES=delay_frames*2
doFilter=False
arg_smooth=0.1
frame_rate=0.5