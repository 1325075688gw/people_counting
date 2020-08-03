# 作者     ：Origin_Point_Cloud
# 创建日期 ：2020-04-09  上午 0:42
# 文件名   ：common.py

from multiprocessing import Queue
import math
import numpy as np
from Origin_Point_Cloud.utils import read_config

#public
queue_for_cluster_transfer = Queue()
queue_for_show_transfer = Queue()
queue_for_door_cart=Queue()
cluster_show_queue = Queue()
frame_data_queue=Queue()
loc_pos = Queue()
point_cloud_show_queue = Queue()

detection_range=6
ymin=0

config=read_config()

#origin point cloud
stop_flag=False


evm_index=[0,1]

ports=[['/dev/ttyACM1','/dev/ttyACM0'],
       ['/dev/ttyACM3','/dev/ttyACM2']]
configuration_files=['ODS_6m_default.cfg','ODS_6m_default.cfg']

save_flag=False