import numpy as np  # 数据结构
import sklearn.cluster as skc
from sklearn.cluster import KMeans
import math
import time
from sklearn.mixture.gaussian_mixture import GaussianMixture
from sklearn import metrics

# 显示聚类点云
import sys

sys.path.append('../')
from common import point_cloud_show_queue
from person import Person
# 显示结束


class UnidentifiedCluster:
    # 保存聚好的类,聚好的类在没有被确认为人之前，就是UnidentifiedCluster
    def __init__(self, points, mix_frame_num, history_data_size):
        self.points = points
        self.points_num = len(points)
        self.center_point = self.compute_center_point()
        self.doppler_v = self.center_point[3]
        self.avg_snr = self.center_point[4]
        self.sum_snr = self.compute_cluster_sum_snr()
        self.length, self.width, self.height = self.compute_cluster_length_width_height()

        # 多帧融合帧数
        self.mix_frame_num = mix_frame_num

        # 这个聚类在过去5帧出现的次数
        self.appear_time = 0
        # 对应过去每一帧的人的下标 list从左到右依次是过去第5帧。。。到最近一帧
        self.match_person_list = [[] for i in range(history_data_size)]

    def compute_cluster_sum_snr(self):
        sum_snr = 0
        for point in self.points:
            sum_snr += point[4]
        return sum_snr

    def compute_center_point(self):
        return np.mean(self.points, axis=0)

    def compute_cluster_length_width_height(self):
        cluster_max = np.max(self.points, axis=0)
        cluster_min = np.min(self.points, axis=0)
        return cluster_max[0] - cluster_min[0], cluster_max[1] - cluster_min[1], cluster_max[2]


class Cluster:
    def __init__(self, eps, minpts, type, min_cluster_count, cluster_snr_limit):
        # 聚类参数：eps:核心点半径，minpts：成为核心点需要的最小点数，type:聚类维度
        self.eps = eps
        self.minpts = minpts
        self.type = type

        # 成为人的聚类最低要求
        self.min_cluster_count = min_cluster_count
        self.cluster_snr_limit = cluster_snr_limit

        # 多帧点云融合
        self.mix_frame_num = 3  # 前后1帧融合，3帧融合；前后2帧融合，5帧融合
        self.frame_data_list = []  # [{"frame_num":1,"points":[]},..]

        # 结果，就是这一帧得到的person list
        self.frame_cluster_result = {'frame_num': 0, 'person_list':[]}

        # 过去n帧的历史结果信息
        self.history_data_size = 5
        self.history_frame_cluster_result = []
        # 过去5帧的搜索区域
        self.search_dist = [0.5, 0.45, 0.4, 0.35, 0.3]

    def do_cluster(self, frame_data):
        # 提取frame_data的帧号和点云数据
        self.frame_cluster_result['frame_num'] = frame_data['frame_num']
        points = Cluster.frame_data_to_cluster_points(frame_data)

        # 进行多帧点云融合，减少空帧，单帧无法得到好的点云的情况
        self.frame_cluster_result['frame_num'], points = self.mix_multi_frame_data(points, frame_data['frame_num'])

        # 加入点云显示队列
        point_cloud_show_queue.put(points)

        # 将点云聚类得到初始聚类结果，UnidentifiedCluster
        unidentified_cluster_list = self.initial_cluster(points)

        # 对初始聚类结果进行分析，对满足人的要求的聚类转换为人，不满足的删除，对一个聚类包含多个人的进行分割
        person_list = self.trans_cluster_to_person(unidentified_cluster_list)

        # 结束这一帧的聚类分析，并保存结果
        self.save_result(person_list)

    @staticmethod
    def frame_data_to_cluster_points(frame_data):
        # 提取每一帧的数据并转换成列表点云的格式
        points = []
        for data in frame_data['point_list']:
            point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
            points.append(point)
        return points

    def mix_multi_frame_data(self, points, frame_num):
        # 进行多帧点云融合
        frame_data_dict = {'frame_num': frame_num, 'points': points}
        self.frame_data_list.append(frame_data_dict)
        if len(self.frame_data_list) > self.mix_frame_num:
            self.frame_data_list.pop(0)
        if len(self.frame_data_list) < self.mix_frame_num:
            return 0, []
        mixed_points = []
        for frame_points_data in self.frame_data_list:
            mixed_points += frame_points_data['points']
        mixed_frame_num = self.frame_data_list[self.mix_frame_num // 2]['frame_num']
        return mixed_frame_num, mixed_points

    def initial_cluster(self, points):
        if len(points) == 0:
            return []
        # 用dbscan对点云聚类，得到聚类标签
        tags = self.dbscan_official(points)
        return self.trans_points_to_unidentified_cluster_list(points, tags)


    def dbscan_official(self, data):
        # 根据类型调用dbscan库
        if data == []:
            return []
        X = np.array(data)
        if self.type == '2D':
            X = X[:, :2]
        elif self.type == '3D':
            X = X[:, :3]
        db = skc.DBSCAN(eps=self.eps, min_samples=self.minpts).fit(X)
        return db.labels_

    def trans_points_to_unidentified_cluster_list(self, points, tags):
        # 过滤噪声，得到分好类的聚类结果
        cluster_tem_dict = self.cluster_by_tag(points, tags)
        self.cluster_filter_by_noise(cluster_tem_dict)

        # 将分好类的聚类结果，转换为UnidentifiedCluster对象，方便之后分析
        unidentified_cluster_list = []
        for i in cluster_tem_dict:
            unidentified_cluster_list.append(UnidentifiedCluster(cluster_tem_dict[i], self.mix_frame_num, self.history_data_size))

        return unidentified_cluster_list

    @staticmethod
    def cluster_by_tag(points, tag):
        # 根据聚类得到的标签对点云分类
        tem_dict = {}
        # 按照类别将点分类
        key_list = []
        for i in tag:
            if i not in key_list:
                key_list.append(i)
        # key_list = sorted(key_list)
        for i in key_list:
            tem_dict[i] = []
        for t, point in zip(tag, points):
            tem_dict[t].append(point)
        return tem_dict

    @staticmethod
    def cluster_filter_by_noise(tem_dict):
        # 去掉噪声
        try:
            tem_dict.pop(-1)
        except:
            pass

    def trans_cluster_to_person(self, unidentified_cluster_list):
        if len(unidentified_cluster_list) == 0:
            return []
        person_list = []

        # 计算每个类在过去出现的次数
        self.cluster_appear_time_in_latest_frame(unidentified_cluster_list)
        # 对每个类依次进行判断，看是否能转换成人，是否需要分割
        for unidentified_cluster in unidentified_cluster_list:
            # 检测这个类是否确定能转换成一个人，不能返回0  能返回1 大于1人返回2
            check_result = Person.check_person(unidentified_cluster, self.min_cluster_count, self.cluster_snr_limit)

            # 如果这个类 无法转换成人 但是在过去有出现过，也将他转换成人
            if check_result == 0 and unidentified_cluster.appear_time >= 1:
                person_list.append(Person(unidentified_cluster))

            # 如果这个类 可以转换成一个人 将其转换
            if check_result == 1:
                person_list.append(Person(unidentified_cluster))

            # 如果这个类无法确定能转换成一个人，类比较大，对其尝试分割
            if check_result == 2:
                divided_cluster_list = self.divide_cluster(unidentified_cluster)
                for cluster in divided_cluster_list:
                    person_list.append(Person(cluster))
        return person_list

    def cluster_appear_time_in_latest_frame(self, cur_frame_cluster_list):
        # 根据最近几帧的聚类结果，对这一帧每一个聚类在过去出现的次数进行统计，并找出这个聚类能被每一帧分配几个人
        if len(self.history_frame_cluster_result) < 5:
            return

        # 通过最近5帧信息，对这一帧的聚类进行过滤
        # search_dist = [0.2, 0.2, 0.25, 0.3, 0.35]

        # i为过去第几帧，j为过去第i帧的第j个人
        for i in range(len(self.history_frame_cluster_result)):
            for j in range(len(self.history_frame_cluster_result[i])):
                # 计算这一帧哪个类，距离第（i,j）个人最近，将这个人的下标赋给它
                min_dist = 1000
                min_dist_index = -1
                for k in range(len(cur_frame_cluster_list)):
                    dist = math.sqrt((self.history_frame_cluster_result[i][j].center_point[0]-cur_frame_cluster_list[k].center_point[0])**2
                                     + (self.history_frame_cluster_result[i][j].center_point[1]-cur_frame_cluster_list[k].center_point[1])**2)
                    if dist < min_dist:
                        min_dist = dist
                        min_dist_index = k
                # 判断这个最近距离是否小于最低匹配距离要求，是，则将人分配给这个类
                if min_dist < self.search_dist[i] and min_dist_index >= 0:
                    cur_frame_cluster_list[min_dist_index].match_person_list[i].append(j)

        # 对这一帧每一个聚类在过去5帧中出现的次数进行统计
        for i in range(len(cur_frame_cluster_list)):
            for j in range(len(cur_frame_cluster_list[i].match_person_list)):
                if len(cur_frame_cluster_list[i].match_person_list[j]) > 0:
                    cur_frame_cluster_list[i].appear_time += 1

    def save_result(self, person_list):
        self.frame_cluster_result['person_list'] = person_list
        self.history_frame_cluster_result.append(person_list)
        if len(self.history_frame_cluster_result) > 5:
            self.history_frame_cluster_result.pop(0)

    def get_height_list(self):
        height_list = []
        for person in self.frame_cluster_result['person_list']:
            height_list.append(person.cur_height)
        return height_list

    def get_cluster_center_point_list(self):
        center_point_list = []
        for person in self.frame_cluster_result['person_list']:
            center_point_list.append([person.center_point[0], person.center_point[1]])
        return center_point_list

    def divide_cluster(self, unidentified_cluster):
        k = 1
        np_points = np.array(unidentified_cluster.points)[:, :2]
        # 如果能用上一帧进行分割，用上一帧的k进行分割
        if self.satisfy_divide_by_last_history_data(unidentified_cluster):
            k = len(unidentified_cluster.match_person_list[self.history_data_size-1])
            print("用上一帧分割",k)
        # 如果不能用上一帧的进行分割，则综合判断点云，找出最合适的k
        else:
            # 根据多个判定条件，得到所有可能的分割数k
            test_k_set = set()

            # 根据人头分割，找到k
            topz_points_k = self.get_k_by_topz_points(unidentified_cluster)
            if topz_points_k >= 2:
                test_k_set.add(topz_points_k)

            # 根据历史出现最多次数，找到k
            history_times_list = self.get_k_by_history_people_num(unidentified_cluster)
            for i in history_times_list:
                test_k_set.add(i)

            # 根据长宽比等先验知识，找到k

            # 依次用高斯混合尝试这些分割，然后选择其中分数最高的k
            score = 0
            people_num = 1
            print(test_k_set)
            for tem_k in test_k_set:
                k_tags = GaussianMixture(n_components=tem_k).fit_predict(np_points)
                try:
                    tem_score = metrics.calinski_harabasz_score(np_points, k_tags)
                except:
                    continue
                if tem_score > score:
                    score = tem_score
                    people_num = tem_k
            k = people_num
        # 当只有一个人时，直接返回
        if k == 1:
            return [unidentified_cluster]
        # 当判断出有k个人时，用高斯混合分割
        print(np_points)
        tags = KMeans(k).fit_predict(np_points)
        # tags = GaussianMixture(n_components=k).fit_predict(np_points)
        return self.trans_points_to_unidentified_cluster_list(unidentified_cluster.points, tags)

    def satisfy_divide_by_last_history_data(self, unidentified_cluster):
        # 将聚类对应上一帧的人的点云合并，然后根据聚类点云和他对应的上一帧的人的总点云相似度，判断是否能分割
        # 找到聚类对应上一帧的人的id
        last_history_person_id = unidentified_cluster.match_person_list[self.history_data_size-1]

        # 如果上一帧只匹配到了小于2人，返回false
        if len(last_history_person_id) < 2:
            return False
        # 将上一帧的人的点云融合
        people_point_cloud = []
        for pid in last_history_person_id:
            people_point_cloud += self.history_frame_cluster_result[self.history_data_size-1][pid].points

        if len(people_point_cloud) == 0:
            return False
        tem_cluster = UnidentifiedCluster(people_point_cloud, self.mix_frame_num, self.history_data_size)

        # 聚类中心点横向距离不超过0.5，纵向距离不超过0.2
        if abs(tem_cluster.center_point[0] - unidentified_cluster.center_point[0]) > 0.5 or abs(
                tem_cluster.center_point[1] - unidentified_cluster.center_point[1]) > 0.2:
            print("不满足聚类横向和纵向距离要求")
            return False

        # 聚类的宽度和长度中，应该和对应人的长，宽类似
        if abs(tem_cluster.length - unidentified_cluster.length) > 0.8 or abs(tem_cluster.width - unidentified_cluster.width) > 0.45:
            print("不满足长宽变化要求，无法分割：长：%f，宽：%f" % (tem_cluster.length - unidentified_cluster.length, tem_cluster.width - unidentified_cluster.width))
            return False
        return True

    def get_k_by_topz_points(self, unidentified_cluster):
        points = unidentified_cluster.points
        k = int(len(points) * 0.1) + 1  # 计算前10%点
        points.sort(key=lambda x: x[2], reverse=True)
        topz_points = [points[i] for i in range(k)]
        X = np.array(topz_points)[:, :2]
        db = skc.DBSCAN(eps=self.eps, min_samples=self.minpts).fit(X)
        tags = db.labels_
        topz_cluster_dict = self.cluster_by_tag(points, tags)
        self.cluster_filter_by_noise(topz_cluster_dict)
        # 如果有人头中心点距离较近，则不进行分割
        center_point_list = []
        for i in topz_cluster_dict:
            center_point = np.mean(topz_cluster_dict[i], axis=0)
            center_point_list.append(center_point)
        # 计算相邻中心点的距离，有距离小于0.5的两个中心点，不进行分割
        dist_list = []
        if len(center_point_list) >= 2:
            for i in range(1, len(center_point_list)):
                dist_list.append(math.sqrt((center_point_list[i - 1][0] - center_point_list[i][0]) ** 2 +
                                           (center_point_list[i - 1][1] - center_point_list[i][1]) ** 2))
        for dist in dist_list:
            if dist < 0.5:
                print("距离太近  不通过人头分")
                return -1
        return len(topz_cluster_dict)

    def get_k_by_history_people_num(self, unidentified_cluster):
        # 找到过去帧中，根据当前类匹配的人数中，出现最多次数的人数
        history_people_num = {}
        for people_id in unidentified_cluster.match_person_list:
            if len(people_id) in history_people_num:
                history_people_num[len(people_id)] += 1
            else:
                history_people_num[len(people_id)] = 0
        times_max = 0
        for i in history_people_num:
            if history_people_num[i] > times_max:
                times_max = history_people_num[i]
        k_list = []
        if times_max == 0:
            return []
        for i in history_people_num:
            if history_people_num[i] == times_max and i > 1:
                k_list.append(i)
        return k_list
