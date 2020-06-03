import queue
import math

loc_pos=queue.Queue()
cluster_points=queue.Queue()
detection_range=8
delay_frames=M=10

xmin=-detection_range/2*math.sqrt(3)
xmax=detection_range/2*math.sqrt(3)
ymin=0
ymax=detection_range