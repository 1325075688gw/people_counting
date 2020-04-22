import numpy as np
import sklearn.cluster as skc
import math


class Cluster():
    def __init__(self, eps, minpts, type, min_cluster_count):
        self.eps = eps
        self.minpts = minpts
        self.type = type
        self.min_cluster_count = min_cluster_count
        self.cluster_division_limit = 80  # 达到80个点，对点云分割（分割失败保持原样）
        self.frame_cluster_dict = {}

    def dbscan_official(self, data):
        """
        使用DBSCAN聚类算法对点云数据进行聚类
        :param data: 点云数据
        :return: 聚类标签
        """
        if data == []:
            return []
        X = np.array(data)
        if self.type == '2D':
            X = X[:, :2]
        elif self.type == '3D':
            X = X[:, :3]
        db = skc.DBSCAN(eps=self.eps, min_samples=self.minpts).fit(X)
        return db.labels_

    def frame_data_to_cluster_points(self, frame_data):
        """
        读取单帧数据，转换为cluster数据结构
        :param frame_data: 每帧的数据
        :return: 每一帧转换后的数据
        """
        points = []
        for data in frame_data['point_list']:
            point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
            points.append(point)
        return points

    def cluster_filter_by_noise(self, temp_dict):
        """
        过滤噪声
        :param temp_dict: 聚类集合
        :return: None
        """
        # 去掉噪声
        try:
            temp_dict.pop(-1)
        except:
            pass

    def cluster_filter_by_count(self, temp_dict, count):
        """
        去除点数较少的类
        :param temp_dict: 聚类集合
        :param count: 聚类所需最少点云数
        :return: None
        """
        del_list = []
        for i in temp_dict:
            if len(temp_dict[i]) < count:
                del_list.append(i)
        for key in del_list:
            temp_dict.pop(key)

    def compute_cluster_height(self, points):
        """
        计算每个聚类的升高
        :param points: 点云
        :return: 各个聚类的身高
        """
        k = int(len(points) * 0.1) + 1  # 计算前10%点
        z_list = [points[i][2] for i in range(k)]
        return round(np.mean(z_list), 3)

    def compute_cluster_length_width(self, points):
        """
        计算每个聚类的长度和宽度
        :param points: 点云
        :return: 各个聚类的长度和宽度
        """
        cluster_max = np.max(points, axis=0)
        cluster_min = np.min(points, axis=0)
        return cluster_max[0] - cluster_min[0], cluster_max[1] - cluster_min[1]

    def compute_cluster_attr(self, temp_dict):
        """
        计算各个聚类对应的属性，参见__init__函数
        :param temp_dict: 聚类集合
        :return: 计算好各个聚类属性后的聚类
        """
        cluster_list = []
        k = 0
        for i in temp_dict:
            cluster_dict = {}
            center_point = np.mean(temp_dict[i], axis=0)
            varlist = np.var(temp_dict[i], axis=0)
            cluster_dict['cluster_id'] = k
            cluster_dict['points_num'] = len(temp_dict[i])
            cluster_dict['center_point'] = [center_point[0], center_point[1], center_point[2]]
            cluster_dict['var_x'] = varlist[0]
            cluster_dict['var_y'] = varlist[1]
            cluster_dict['var_z'] = varlist[2]
            cluster_dict['height'] = self.compute_cluster_height(temp_dict[i])
            cluster_dict['points'] = temp_dict[i]
            cluster_dict['length'], cluster_dict['width'] = self.compute_cluster_length_width(temp_dict[i])
            cluster_list.append(cluster_dict)
            k += 1
        return cluster_list

    def cluster_by_tag(self, points, tag):
        """
        根据DBSCAN算法返回的结果标签，将点云和聚类进行对应
        :param points: 点云
        :param tag: DBSCAN返回的标签结果
        :return: 点云和聚类对应好的聚类集合
        """
        temp_dict = {}
        key_list = []
        for i in tag:
            if i not in key_list:
                key_list.append(i)
        for i in key_list:
            temp_dict[i] = []
        for t, point in zip(tag, points):
            temp_dict[t].append(point)
        return temp_dict

    def divide_cluster_by_count_doppler(self, temp_dict):
        """
        根据doppler将两个靠的比较近的人进行分割
        :param temp_dict: 聚类集合
        :return:None
        """
        i = 0
        n = len(temp_dict)
        new_key = n
        while i < n:
            if len(temp_dict[i]) > 100:
                print(len(temp_dict[i]))
                X = np.array(temp_dict[i])
                # dbscan至少2维，这里给他主动将snr那一列设为同样的值，不影响聚类结果
                for x_array in X:
                    x_array[4] = 0
                X = X[:, 3:5]
                db = skc.DBSCAN(eps=0.2, min_samples=15).fit(X)
                divided_dict = self.cluster_by_tag(temp_dict[i], db.labels_)
                self.cluster_filter_by_noise(divided_dict)
                length_divided = len(divided_dict)
                self.cluster_filter_by_count(divided_dict, 40)
                for j in divided_dict:
                    temp_dict[j + new_key] = divided_dict[j]
                new_key += length_divided
                # 删除分割的类
                temp_dict.pop(i)
            i += 1

    def cluster_filter_by_count_and_distance(self, temp_dict):
        """
        根据距离和点云数量，将不达标的聚类清理掉
        :param temp_dict: 聚类集合
        :return: None
        """
        del_list = []
        for i in temp_dict:
            center_point = np.mean(temp_dict[i], axis=0)
            dist = math.sqrt(center_point[0] * center_point[0] + center_point[1] * center_point[1])
            if dist < 7 and len(temp_dict[i]) < (1 - dist / 7) * self.min_cluster_count:
                del_list.append(i)
        for key in del_list:
            temp_dict.pop(key)

    def cluser_to_person(self, points, tag):
        """
        将聚类结果转换为Person
        :param points: 点云
        :param tag: DBSCAN返回的聚类标签
        :return: 人列表
        """
        temp_dict = self.cluster_by_tag(points, tag)
        self.cluster_filter_by_noise(temp_dict)
        self.cluster_filter_by_count_and_distance(temp_dict)
        person_list = self.compute_cluster_attr(temp_dict)
        return person_list

    def get_cluster_center_point_list(self):
        """
        获取每个聚类的中心
        :return: 聚类中心列表
        """
        cluster_center_point_list = []
        for cluster in self.frame_cluster_dict['cluster']:
            center_point = [cluster['center_point'][0], cluster['center_point'][1]]
            cluster_center_point_list.append(center_point)
        return cluster_center_point_list

    def get_height_list(self):
        """
        获取人的身高
        :return: 每一帧人的身高列表
        """
        cluster_height_list = []
        for cluster in self.frame_cluster_dict['cluster']:
            height = cluster['height']
            cluster_height_list.append(height)
        return cluster_height_list

    def do_clsuter(self, frame_data):
        """
        完成聚类和人体识别
        :param frame_data: 每一帧的数据
        :return: 聚好类的人
        """
        self.frame_cluster_dict['frame_num'] = frame_data['frame_num']
        points = self.frame_data_to_cluster_points(frame_data)
        tag = self.dbscan_official(points)
        self.frame_cluster_dict['cluster'] = self.cluser_to_person(points, tag)