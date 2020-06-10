from origin_point_cloud import common

import math

# 判断轨迹是否位于边缘
def is_at_edge_out(location):
    x = location[0]
    y = location[1]

    if x < common.xmin + 0.5 or x > common.xmax - 0.5 or y < common.ymin+2 or y > common.ymax - 2:
        return True

    return False

def is_at_edge_in(location):
    x = location[0]
    y = location[1]

    if x < common.xmin + 1 or x > common.xmax - 1 or y < common.ymin + 2.5 or y > common.ymax - 2.5:
        return True

    return False

#判断点是否已经超出了给定范围
def is_out_area(location):
    # 判断是否位于给定空间之外
    if location[0] <= common.xmin or location[0] >= common.xmax or location[1] > common.ymax or location[1]<common.ymin:
        return True
    # 判断是否位于雷达探测范围之外
    if (math.sqrt(3) * location[1] - abs(location[0])) / 2 < 0:
        return True
    return False