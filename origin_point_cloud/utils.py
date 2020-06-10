import numpy as np
import math

from origin_point_cloud import common

def get_coordinate_in_radar_num(radar_num, points):
    radar_pos=np.array(common.relative_poses[radar_num])
    direction=np.array(common.directions[radar_num])
    xdirection=np.array(common.xdirections[radar_num])
    nppoints=np.array(points)

    relative_loc=nppoints[:,:2]-radar_pos

    nppoints[:,:2]=relative_loc.dot([xdirection/np.linalg.norm(xdirection),direction/np.linalg.norm(direction)])

    return nppoints

def get_radar_num(x,y):
    '''
    :function:根据雷达摆放位置高度定制化
    '''
    if y<common.ymax/2:
        return 0
    else:
        return 1

# def get_radar_num(x,y):
#     '''
#     :note:三块雷达，顺序分别为左下，中间和右下
#     :require:angle1,angle2
#     '''
#     angle1=0
#     angle2=0
#     slope1=math.tan(angle1)
#     slope2=math.tan(angle2)
#
#     if y>slope1*(x-common.relative_poses[0][0]):
#         return 1
#     elif y<slope2*(x-common.relative_poses[2][0]):
#         return 2
#     else:
#         return 3