import numpy as np
import sklearn.cluster as skc
import json
import os

# 判定范围
x_min = -2
x_max = 2
y_min = 5
y_max = 7


def center_point_in_range(center_point):
    if len(center_point) == 0:
        return False
    global x_min
    global x_max
    global y_min
    global y_max
    if x_min < center_point[0] < x_max and y_min < center_point[1] < y_max:
        return True
    return False


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


def get_offset(frame_data, standard_radar_num, adjusted_radar_num):
    # 根据标准帧数据，矫正其他雷达的帧数据

    # 获取每一帧的点云数据
    standard_frame_list = []
    adjusted_frame_list = []
    for i in frame_data:
        standard_frame_list.append(frame_data[i]['point_list'][standard_radar_num])
        adjusted_frame_list.append(frame_data[i]['point_list'][adjusted_radar_num])

    # 计算上述2个列表的最小长度
    length = len(standard_frame_list)

    # 对每一帧的两组点云分别聚类得到中心点，并算出偏移值，加入偏移值队列
    offset_list = []
    for i in range(length):
        standard_center = get_cluster_center(standard_frame_list[i])
        adjusted_center = get_cluster_center(adjusted_frame_list[i])
        # 判断中心点是否在指定区域
        if not center_point_in_range(standard_center) or not center_point_in_range(adjusted_center):
            continue
        # 如果算出来这一帧的2个中心点，将他们的偏移值加入列表
        if not (standard_center == []) and not (adjusted_center == []):
            offset_list.append([standard_center[0]-adjusted_center[0],standard_center[1]-adjusted_center[1]])

    # 计算偏移值的平均值，并返回
    offset = [0, 0]
    if not (offset_list == []):
        np_offset = np.mean(offset_list, axis=0)
        offset[0] = np_offset[0]
        offset[1] = np_offset[1]
    return offset


def run_adjust(tem_file):
    offset = get_offset(tem_file, '0', '1')
    print([offset[0], offset[1]])


if __name__ == '__main__':
    filename = './data/gzz/data_6_11,单人绕圈走，两块板子，第1次/cart_data.json'
    with open(filename, 'r', encoding='utf-8') as f:
        tem_file = json.load(f)
    offset = get_offset(tem_file, '0', '1')
    print([offset[0],offset[1]])
