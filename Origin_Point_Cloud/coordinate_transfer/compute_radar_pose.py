import math
import numpy as np
import sklearn.cluster as skc
import json
import sys
from sklearn import linear_model

sys.path.append("../")

from coordinate_transfer.radar_coordinate_transfer import radar2_direction_in_global, compute_direction_vector, compute_relative_position, radar2_position_in_global
from Cluster import cluster_common
from coordinate_transfer.pos_direct_modify import cal_radar2_params


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
    position_list = []

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
        dist1 = math.sqrt((radar1_center1[0]-radar1_center2[0])**2+(radar1_center1[1]-radar1_center2[1])**2)
        dist2 = math.sqrt((radar2_center1[0]-radar2_center2[0])**2+(radar2_center1[1]-radar2_center2[1])**2)
        if abs(dist1-dist2) > 0.05:
            continue
        #print(math.sqrt((radar1_center1[0]-radar1_center2[0])**2+(radar1_center1[1]-radar1_center2[1])**2),math.sqrt((radar2_center1[0]-radar2_center2[0])**2+(radar2_center1[1]-radar2_center2[1])**2))
        tem_dv = compute_direction_vector(radar1_center2[0]-radar1_center1[0], radar1_center2[1]-radar1_center1[1],
                                          radar2_center2[0]-radar2_center1[0], radar2_center2[1]-radar2_center1[1])
        direction_vector_list.append(tem_dv)
        position_list.append(compute_relative_position(radar1_center1[0],radar1_center1[1],tem_dv[0],tem_dv[1],radar2_center1[0],radar2_center1[1]))
        position_list.append(compute_relative_position(radar1_center2[0],radar1_center2[1],tem_dv[0],tem_dv[1],radar2_center2[0],radar2_center2[1]))
    #print(position_list)
    return np.mean(direction_vector_list, axis=0), np.mean(position_list, axis=0)



# 训练人的特征

def get_spin_theta(center_point):
    # 求将y的正半轴的中心点旋转到y的正半轴上，需要旋转的角度（用弧度表示）
    theta = 0
    if center_point[0] < 0:
        theta = -1 * math.atan(center_point[1] / center_point[0]) - math.pi / 2
    if center_point[0] > 0:
        theta = math.pi / 2 - math.atan(center_point[1] / center_point[0])
    return theta


def transfer_point(x, y, theta):
    # 将点逆时针旋转theta角度，返回旋转后的二维坐标
    t_x = math.cos(theta) * x - math.sin(theta) * y
    t_y = math.cos(theta) * y + math.sin(theta) * x
    return [t_x, t_y]


def compute_cluster_length_width_height(points, center_point):
    spin_theta = get_spin_theta(center_point)
    cluster_max = np.max(points, axis=0)
    cluster_min = np.min(points, axis=0)
    if abs(spin_theta) < 0.05:
        return cluster_max[0] - cluster_min[0], cluster_max[1] - cluster_min[1], cluster_max[2]

    # 对坐标进行旋转
    trans_points = []
    for point in points:
        trans_point = transfer_point(point[0], point[1], spin_theta)
        trans_points.append(trans_point)

    # 求旋转后的点云长宽
    trans_cluster_max = np.max(trans_points, axis=0)
    trans_cluster_min = np.min(trans_points, axis=0)
    # print("转换后的长：%f,宽：%f"%(trans_cluster_max[0] - trans_cluster_min[0], trans_cluster_max[1] - trans_cluster_min[1]))
    return trans_cluster_max[0] - trans_cluster_min[0], trans_cluster_max[1] - trans_cluster_min[1], cluster_max[2]


def read_and_cluster(radar_data):
    length_list = []
    width_list = []
    points_num_list = []
    mix_points_list = []
    dist_list = []
    mix_num = cluster_common.mixed_num
    for k in radar_data:
        frame_data = radar_data[k]

        #数据转换
        points = frame_data['point_list']
        # for data in frame_data['point_list']:
        #     point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
        #     #if 7 > point[1] > 2:
        #     points.append(point)

        mix_points_list.append(points)
        if len(mix_points_list) < mix_num:
            continue
        if len(mix_points_list) > mix_num:
            mix_points_list.pop(0)

        mix_points = []

        for ps in mix_points_list:
            mix_points += ps

        if mix_points == []:
            continue
        #聚类
        X = np.array(mix_points)
        X = X[:, :2]
        db = skc.DBSCAN(eps=0.25, min_samples=5).fit(X)
        tags = db.labels_

        #分类
        tem_dict = {}
        # 按照类别将点分类
        key_list = []
        for i in tags:
            if i not in key_list:
                key_list.append(i)
        for i in key_list:
            tem_dict[i] = []
        for t, point in zip(tags, mix_points):
            tem_dict[t].append(point)

        #滤波
        del_list = []
        for i in tem_dict:
            if i == -1 or len(tem_dict[i]) < 20:
                del_list.append(i)

        for i in del_list:
            tem_dict.pop(i)

        #求中心点长宽点数
        # 选择聚类中点数最多的聚类
        proper_cluster_key = 0
        nums = 0
        for i in tem_dict:
            if len(tem_dict[i]) > nums:
                nums = len(tem_dict[i])
                proper_cluster_key = i
        if nums > 0:
            cluster_center_point = np.mean(tem_dict[proper_cluster_key], axis=0)
            dist_list.append(math.sqrt(cluster_center_point[0]**2+cluster_center_point[1]**2))
            length, width, height = compute_cluster_length_width_height(tem_dict[proper_cluster_key], cluster_center_point)
            length_list.append(length)
            # print(length)
            width_list.append(width)
            points_num_list.append(len(tem_dict[proper_cluster_key]))

    avg_length = np.mean(length_list)
    avg_width = np.mean(width_list)
    avg_points = np.mean(points_num_list)

    # 对距离和长度做线性回归，找到它们的关系
    x_dist = [[dist] for dist in dist_list]
    y_length = [[length] for length in length_list]
    model = linear_model.LinearRegression()
    model.fit(x_dist, y_length)
    # print("斜率：%f,截距：%f"% (model.coef_, model.intercept_))
    # print(model.predict([[0]]))
    # print("得分:%f" % model.score(x_dist, y_length))
    length_predict = model.predict(x_dist)
    return [avg_length, avg_width, avg_points, model.coef_, model.intercept_, model.score(x_dist, y_length)]


def training_data(radar_data):
    person_attr = read_and_cluster(radar_data)
    return [person_attr[3][0][0],person_attr[4][0], person_attr[1]+0.1, person_attr[1]-0.05, person_attr[2]]


def run(radar1_x,radar1_y,x1,y1,dir_path):
    # (x1,y1)为雷达1在世界坐标下的方向向量
    filename1 = dir_path + "/0/cart_data.json"
    filename2 = dir_path + "/1/cart_data.json"
    with open(filename1, 'r', encoding='utf-8') as f:
        radar1_data = json.load(f)
    with open(filename2, 'r', encoding='utf-8') as f:
        radar2_data = json.load(f)
    global_direction,global_position=cal_radar2_params(radar1_x,radar1_y,x1,y1,dir_path)
    print('雷达2在世界坐标系下的位置和方向向量计算完毕')
    # 运行训练人的特征
    radar1_person_feature=training_data(radar1_data)
    radar2_person_feature=training_data(radar2_data)
    print('雷达1和雷达2照射的人的点云特征计算完毕')
    return global_direction,global_position,radar1_person_feature,radar2_person_feature

def compute_radar1_direction(filename):
    filename1 = filename + "/0/cart_data.json"
    with open(filename1, 'r', encoding='utf-8') as f:
        radar1_data = json.load(f)

    radar1_center_list = get_cluster_center_list(radar1_data)
    radar1_direction_list = []
    for center in radar1_center_list:
        if center == []:
            continue
        d = math.sqrt(center[0]**2+center[1]**2)
        radar1_direction_list.append(compute_direction_vector(d,0,center[0],center[1]))

    radar1_direction = np.mean(radar1_direction_list,axis=0)
    return radar1_direction


def run_one_radar(dir_path):
    filename1 = dir_path + "/0/cart_data.json"
    with open(filename1, 'r', encoding='utf-8') as f:
        radar1_data = json.load(f)
    print("radar1:")
    training_data(radar1_data)


def training_radar3(dir_path):
    filename = dir_path + "/2/cart_data.json"
    with open(filename, 'r', encoding='utf-8') as f:
        radar_data = json.load(f)
    print("radar3:")
    training_data(radar_data)

if __name__ == "__main__":
    dir_path = "../data/"
    filename = "radar_data"
    path = dir_path + filename

    # 求雷达1的方向向量
    # compute_radar1_direction(path)
    # 输入雷达1的位置和方向向量, 计算雷达2的位置，方向，和雷达1,2的训练属性
    print(run(0, 0, -4.5, 5, path))

    # 训练雷达3
    # training_radar3(path)

    # 只有一个雷达训练
    # run_one_radar(filename)
