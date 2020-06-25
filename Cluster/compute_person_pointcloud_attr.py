import os
import numpy as np
import json
import sklearn.cluster as skc
import math
import matplotlib.pyplot as plt
from sklearn import linear_model


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


def read_and_cluster(filename):
    length_list = []
    width_list = []
    points_num_list = []
    mix_points_list = []
    dist_list = []
    mix_num = 3
    #f = open(filename, 'r', encoding='utf-8')
    with open(filename, 'r', encoding='utf-8') as f:
        temfile = json.load(f)
        # print(temfile)
        for k in temfile:
            frame_data = temfile[k]

            #数据转换
            points = []
            for data in frame_data['point_list']:
                point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
                if 7 > point[1] > 2:
                    points.append(point)

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
            for i in tem_dict:
                cluster_center_point = np.mean(tem_dict[i], axis=0)
                dist_list.append(math.sqrt(cluster_center_point[0]**2+cluster_center_point[1]**2))
                length, width, height = compute_cluster_length_width_height(tem_dict[i], cluster_center_point)
                length_list.append(length)
                width_list.append(width)
                points_num_list.append(len(tem_dict[i]))

    avg_length = np.mean(length_list)
    avg_width = np.mean(width_list)
    avg_points = np.mean(points_num_list)
    plt.figure(figsize=(10,5))
    plt.scatter(dist_list, length_list,marker='.')

    # 对距离和长度做线性回归，找到它们的关系
    x_dist = [[dist] for dist in dist_list]
    y_length = [[length] for length in length_list]
    model = linear_model.LinearRegression()
    model.fit(x_dist, y_length)
    # print("斜率：%f,截距：%f"% (model.coef_, model.intercept_))
    # print(model.predict([[0]]))
    # print("得分:%f" % model.score(x_dist, y_length))
    length_predict = model.predict(x_dist)
    plt.plot(x_dist, length_predict, 'g-')
    plt.show()
    return [avg_length, avg_width, avg_points, model.coef_, model.intercept_, model.score(x_dist, y_length)]


def run_training_data(base_path, path, filename, result_list):
    #print
    if filename == 'cart_transfer_data.json':
        #print(os.path.join(base_path, path, filename))
        result_list.append(read_and_cluster(os.path.join(base_path, path, filename)))
        #print(result_list)
        return
    try:
        next_file_names = os.listdir(os.path.join(base_path, path, filename))
        #print(next_file_names)
        for next_file_name in next_file_names:
            run_training_data(base_path, os.path.join(path, filename), next_file_name, result_list)
    except:
        pass
        #print(filename + "该文件不符合要求")


if __name__ == "__main__":
    base_path = '.\\training_data'  #不变
    path = ''  #调整这个参数
    filename = ''
    coef = 0 # 斜率
    intercept = 0  # 截距
    score = 0
    result_list = []
    run_training_data(base_path, path, filename, result_list)
    for result in result_list:
        if result[5] > score:
            score = result[5]
            coef = result[3]
            intercept = result[4]
    person_attr = np.mean(result_list, axis=0)
    print("person_length_max：%f,person_length_min：%f,person_width_max:%f,person_width_min:%f,person_points:%f" % (person_attr[0]+0.35, person_attr[0]-0.05, person_attr[1]+0.1, person_attr[1]-0.05, person_attr[2]))
    print("长度方程：斜率：%f, 截距:%f" % (coef, intercept))
