import math

from Origin_Point_Cloud import common

# 判断轨迹是否位于边缘
def is_at_edge_out(location):
    x = location[0]
    y = location[1]

    if x < common.xmin + 0.5 or x > common.xmax - 0.5 or y < common.ymin+1 or y > common.ymax - 1:
        return True

    if (y-x)*math.tan(common.azi_range)<0.5:
        return True

    return False

def is_out_area(location):
    if location[0]<common.xmin-0.5 or location[0]>=common.xmax+0.5 or location[1]>common.ymax+0.5 or location[1]<common.ymin-0.5:
        return True

    if (location[0]-location[1])*math.tan(common.azi_range)>0.5:
        return True

    return False