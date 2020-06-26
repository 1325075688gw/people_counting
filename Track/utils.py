from Origin_Point_Cloud import common

import math

# 判断轨迹是否位于边缘
def is_at_edge_out(location):
    x = location[0]
    y = location[1]

    if x < common.xmin + 0.5 or x > common.xmax - 0.5 or y < common.ymin+1 or y > common.ymax - 1:
        return True

    return False

def is_at_edge_in(location):
    x = location[0]
    y = location[1]

    if x < common.xmin + 1 or x > common.xmax - 1 or y < common.ymin + 2.5 or y > common.ymax - 2.5:
        return True

    return False

def is_out_area(location):
    return False
    if location[0]<common.xmin-0.5 or location[0]>=common.xmax+0.5 or location[1]>common.ymax+0.5 or location[1]<common.ymin-0.5:
        return True
    return False