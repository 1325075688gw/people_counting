import queue,math

loc_pos=queue.Queue()
MAX_SAVE_FRAMES=150
arg_smooth=0.1
detection_range=8
xmin=-detection_range/2*math.sqrt(3)
xmax=detection_range/2*math.sqrt(3)
ymin=1
ymax=detection_range
delay_frames=30
frame_rate=0.5