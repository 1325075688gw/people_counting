from Origin_Point_Cloud import common

# 判断轨迹是否位于边缘
def is_at_edge_out(location):
    x = location[0]
    y = location[1]

    if x>common.xmax-3 and y>common.ymax-1.5:
        return True

    return False

def is_at_edge_in(location):
    x = location[0]
    y = location[1]

    if x<common.xmin+3 and y<common.ymin+1.5:
        return True

    return False

def is_out_area(location):
    if location[0]<common.xmin-0.5 or location[0]>=common.xmax+0.5 or location[1]>common.ymax+0.5 or location[1]<common.ymin-0.5:
        return True
    return False