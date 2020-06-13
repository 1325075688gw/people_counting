import math
import numpy as np
import sklearn.cluster as skc
import json
from radar_coordinate_transfer import radar2_direction_in_global, compute_direction_vector


# 聚类求解
def get_cluster_center(points):
    if len(points) == 0:
        return []
    # 聚类
    X = np.array(points)
    X = X[:, :2]
    db = skc.DBSCAN(eps=0.25, min_samples=5).fit(X)
    tags = db.labels_

    # 分类
    tem_dict = {}
    # 按照类别将点分类
    key_list = []
    for i in tags:
        if i not in key_list:
            key_list.append(i)
    for i in key_list:
        tem_dict[i] = []
    for t, point in zip(tags, points):
        tem_dict[t].append(point)

    # 滤波
    del_list = []
    for i in tem_dict:
        if i == -1 or len(tem_dict[i]) < 20:
            del_list.append(i)

    for i in del_list:
        tem_dict.pop(i)

    # 如果有多个聚类结果 取点数最多的作为代表
    points_tem_max = 0
    index = -1
    for i in tem_dict:
        if len(tem_dict) > points_tem_max:
            points_tem_max = len(tem_dict)
            index = i

    center = []
    if points_tem_max > 0:
        cluster_center_point = np.mean(tem_dict[index], axis=0)
        center = [cluster_center_point[0], cluster_center_point[1]]
    return center


def get_cluster_center_list(radar_data):
    center_list = []
    for i in radar_data:
        points = radar_data[i]['point_list']
        center_list.append(get_cluster_center(points))
    return center_list


def compute_radar_direction_vector(radar1_data, radar2_data):
    # 求雷达2相对于雷达1的方向向量
    # 方向向量列表
    direction_vector_list = []

    radar1_center_list = get_cluster_center_list(radar1_data)
    radar2_center_list = get_cluster_center_list(radar2_data)
    min_length = min(len(radar1_center_list), len(radar2_center_list))
    # 将中心点分成前后2部分，2部分组成向量，根据这个向量完成方向向量计算
    mid_index = min_length//2
    i = 0
    j = mid_index
    while i < mid_index:
        # 求雷达1的方向向量
        radar1_center1 = radar1_center_list[i]
        radar1_center2 = radar1_center_list[j]
        radar2_center1 = radar2_center_list[i]
        radar2_center2 = radar2_center_list[j]
        i += 1
        j += 1
        if radar1_center1 == [] or radar1_center2 == [] or radar2_center1 == [] or radar2_center2 == []:
            continue
        direction_vector_list.append(compute_direction_vector(radar1_center2[0]-radar1_center1[0], radar1_center2[1]-radar1_center1[1],
                                                              radar2_center2[0]-radar2_center1[0], radar2_center2[1]-radar2_center1[1]))
    return np.mean(direction_vector_list, axis=0)


def run(x1,y1):
    # (x1,y1)为雷达1在世界坐标下的方向向量
    dir_path = "../training_data/data_6_12,单人前后走,未转换,第6次"
    filename1 = dir_path + "/0/cart_data.json"
    filename2 = dir_path + "/1/cart_data.json"
    with open(filename1, 'r', encoding='utf-8') as f:
        radar1_data = json.load(f)
    with open(filename2, 'r', encoding='utf-8') as f:
        radar2_data = json.load(f)
    radar2_direction_in_radar1 = compute_radar_direction_vector(radar1_data, radar2_data)
    print(radar2_direction_in_radar1)
    res = radar2_direction_in_global(x1, y1, radar2_direction_in_radar1[0], radar2_direction_in_radar1[1])
    print(res)


run(1,1)
