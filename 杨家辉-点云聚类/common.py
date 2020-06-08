from queue import Queue

# from multiprocessing import Queue

frame_data_queue = Queue() 
cluster_show_queue = Queue()
point_cloud_show_queue = Queue()
queue_for_cluster_transfer = Queue()

divide_line = 6  # 2个雷达的分界线位置
radar_2_x = 0  # 雷达2在雷达1下的x坐标
radar_2_y = 12  # 雷达2在雷达1下的y坐标