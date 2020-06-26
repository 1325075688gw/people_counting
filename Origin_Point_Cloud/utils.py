import numpy as np
import math
import copy
from scipy.optimize import linear_sum_assignment

from Origin_Point_Cloud import common

def get_coordinate_in_radar_num(radar_num, points):
    radar_index=common.evm_index[radar_num]

    radar_pos=np.array(common.relative_poses[radar_index])
    direction=np.array(common.directions[radar_index])
    xdirection=np.array(common.xdirections[radar_index])
    nppoints=np.array(points)

    relative_loc=nppoints[:,:2]-radar_pos

    nppoints[:,0]=relative_loc.dot(xdirection/np.linalg.norm(xdirection))
    nppoints[:,1]=relative_loc.dot(direction/np.linalg.norm(direction))

    return nppoints

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

# def mix_radar_clusters(cluster_centers,people_height_list):
#     '''
#     将公共区域的不同雷达的点进行配对
#     :param cluster_centers: 字典，键为雷达，值为对应雷达聚类结果(list)
#     :param people_height_list: 字典，键为雷达，值为对应聚类结果身高(list)
#     :return:一个点集
#     '''
#     centers=[]
#     heights=[]
#
#     for radar in cluster_centers:
#         centers.extend(cluster_centers[radar])
#         heights.extend(people_height_list[radar])
#
#     return centers,heights

def mix_radar_clusters(cluster_centers,people_height_list):
    '''
    将公共区域的不同雷达的点进行配对
    :param cluster_centers: 字典，键为雷达，值为对应雷达聚类结果(list)
    :param people_height_list: 字典，键为雷达，值为对应聚类结果身高(list)
    :return:一个点集
    '''
    centers=[]
    heights=[]

    common_area_dict=dict()
    for i in cluster_centers:
        for center,height in zip(cluster_centers[i],people_height_list[i]):
            if np.linalg.norm(center-common.relative_poses[int(i)])>common.detection_range:
                continue
            radar_indexes=get_radar_nums_common(center[0],center[1])
            key=''
            for radar_index in radar_indexes:
                key+=str(radar_index)
            if key in common_area_dict:
                if i in common_area_dict[key]:
                    common_area_dict[key][i].append(np.array([center[0],center[1],height]))
                else:
                    common_area_dict[key][i]=[np.array([center[0],center[1],height])]
            else:
                common_area_dict[key]=dict()
                common_area_dict[key][i]=[np.array([center[0],center[1],height])]
    for key in common_area_dict:
        points=np.array(assign_common_area_points(common_area_dict[key]))
        if len(points)==0:
            continue
        centers.extend(points[:,:2])
        heights.extend(points[:,2])
    return centers,heights

def assign_common_area_points(common_area_radar_dict):
    '''
    对多块雷达照射的公共区域中的点集进行配对
    :return: 一个点集
    '''
    radar_points=list(common_area_radar_dict.values())
    while True:
        if len(radar_points)==1:
            return radar_points[0]
        points=assign_common_two_area_points(radar_points[0],radar_points[1])
        del radar_points[0]
        del radar_points[0]
        radar_points.append(points)

def cal_distances(points1,points2):
    '''
    计算两个点集的两两之间的距离用于配对
    '''
    distances = []
    for point1 in points1:
        row = []
        for point2 in points2:
            row.append(np.linalg.norm(point2 - point1))
        distances.append(row)

    return np.array(distances)

def assign_common_two_area_points(points1,points2):
    '''
    对两个点集进行配对
    :return: 一个点集
    '''
    points=[]
    distances=cal_distances(points1,points2)
    row_ind,col_ind=get_assignment_min_distance(distances)

    indexi=[]
    indexj=[]
    for i in row_ind:
        j=col_ind[i]
        indexi.append(i)
        indexj.append(j)
        if distances[i][j] < common.max_accept_pair_distance:
            points.append((points1[i]+points2[j])/2)
        else:
            points.append(points1[i])
            points.append(points2[j])

    #将未配对的点加入点集中
    for i in range(len(points1)):
        if i not in indexi:
            points.append(points1[i])
    for j in range(len(points2)):
        if j not in indexj:
            points.append(points2[j])
    '''
    :NOTE:使用匈牙利算法对点进行分配后得到匹配后的点
    # row_ind, col_ind = get_assignment_hungary(distances)
    index_j=[]
    # for i in range(len(points1)):
    #     j=col_ind[i]
    #     index_j.append(j)
    #     if j<len(points2):
    #         #对被配对的点做判断，距离过远则认为是两个人
    #         if distances[i][j]<common.max_accept_pair_distance:
    #             points.append((points1[i]+points2[j])/2)
    #         else:
    #             points.append(points1[i])
    #             points.append(points2[j])
    #     else:
    #         points.append(points1[i])
    # #处理points2中的点比points1中点多的情况
    # for j in col_ind:
    #     if j<len(points2) and j not in index_j:
    #         points.append(points2[j])
    '''

    return points

def get_assignment_hungary(distanceMatrix):
    '''
    使用匈牙利算法获得点与轨迹的匹配结果
    '''
    row = len(distanceMatrix)
    col = len(distanceMatrix[0])
    matrix = copy.copy(distanceMatrix)

    while row != col:
        if row < col:
            sub_row = np.array([100.0] * col)
            matrix = np.append(matrix, [sub_row], axis=0)
            row += 1
        else:
            sub_col = np.array([100.0] * row)
            matrix = np.append(matrix.T, [sub_col], axis=0).T
            col += 1

    row_ind, col_ind = linear_sum_assignment(matrix)

    return row_ind, col_ind

# 使用最短距离算法获得点与轨迹的匹配结果
def get_assignment_min_distance(distanceMatrix):
    row_ind=[]
    col_ind=dict()

    if len(distanceMatrix)==0:
        return row_ind,col_ind

    while True:
        if len(col_ind)==len(distanceMatrix) or len(col_ind)==len(distanceMatrix[0]):
            break
        indexi=-1
        indexj=-1
        min_distance=float('inf')
        for i in range(len(distanceMatrix)):
            if i in col_ind:
                continue
            for j in range(len(distanceMatrix[i])):
                if j in col_ind.values():
                    continue
                if distanceMatrix[i][j]<=min_distance:
                    min_distance=distanceMatrix[i][j]
                    indexi=i
                    indexj=j
        if indexi!=-1 and indexj!=-1:
            col_ind[indexi]=indexj
            row_ind.append(indexi)

    return row_ind,col_ind

def get_radar_nums_common(x,y):
    '''
    根据点的坐标给出能照射到之的雷达，可扩展至N块板子
    '''
    radar_index=[]

    for i in common.evm_index:
        location=np.array([x,y])
        radar_pos=common.relative_poses[i]
        direction=common.directions[i]

        relative_pos=location-radar_pos
        if np.linalg.norm(relative_pos)<common.detection_range:
            if math.acos(relative_pos.dot(direction)/(np.linalg.norm(relative_pos)*np.linalg.norm(direction)))<=common.azi_range:
                radar_index.append(i)

    return radar_index