import numpy as np  # 数据结构
import sklearn.cluster as skc
import math
from sklearn.mixture.gaussian_mixture import GaussianMixture

# 显示聚类点云
import sys

sys.path.append('../')
from Cluster.person import Person
from Origin_Point_Cloud.utils import get_coordinate_in_radar_num
# 显示结束
from Cluster import cluster_common

class UnidentifiedCluster:
    # 保存聚好的类,聚好的类在没有被确认为人之前，就是UnidentifiedCluster
    def __init__(self, points, mix_frame_num, history_data_size, radar_index):
        self.points = points
        self.points_num = len(points)
        self.center_point = self.compute_center_point(self.points)
        self.dist = math.sqrt(self.center_point[0]**2+self.center_point[1]**2)
        self.doppler_v = self.center_point[3]
        self.avg_snr = self.center_point[4]
        self.sum_snr = self.compute_cluster_sum_snr()

        # 多帧融合帧数
        self.mix_frame_num = mix_frame_num

        # 这个聚类在过去5帧出现的次数
        self.appear_time = 0
        # 对应过去每一帧的人的下标 list从左到右依次是过去第5帧。。。到最近一帧
        self.match_person_list = [[] for i in range(history_data_size)]

        # 用于多块雷达的参数

        self.radar_num = radar_index  # 聚类属于哪个雷达探测范围
        if self.radar_num == 2:
            self.origin_radar_points = self.points
        else:
            self.origin_radar_points = get_coordinate_in_radar_num(self.radar_num, self.points)  # 在原始雷达下的点云坐标
        self.origin_radar_center_point = self.compute_center_point(self.origin_radar_points)  # 在原始雷达下的中心坐标
        self.origin_radar_dist = math.sqrt(self.origin_radar_center_point[0]**2+self.origin_radar_center_point[1]**2)
        self.length, self.width, self.height = self.compute_cluster_length_width_height(self.origin_radar_points, self.origin_radar_center_point)

    def compute_cluster_sum_snr(self):
        sum_snr = 0
        for point in self.points:
            sum_snr += point[4]
        return sum_snr

    @staticmethod
    def compute_center_point(points):
        return np.mean(points, axis=0)

    @staticmethod
    def compute_cluster_length_width_height(points, center_point):
        spin_theta = UnidentifiedCluster.get_spin_theta(center_point)
        cluster_max = np.max(points, axis=0)
        cluster_min = np.min(points, axis=0)
        if abs(spin_theta) < 0.05:
            return cluster_max[0] - cluster_min[0], cluster_max[1] - cluster_min[1], cluster_max[2]

        # 对坐标进行旋转
        trans_points = []
        for point in points:
            trans_point = UnidentifiedCluster.transfer_point(point[0], point[1], spin_theta)
            trans_points.append(trans_point)

        # 求旋转后的点云长宽
        trans_cluster_max = np.max(trans_points, axis=0)
        trans_cluster_min = np.min(trans_points, axis=0)
        #print("转换后的长：%f,宽：%f"%(trans_cluster_max[0] - trans_cluster_min[0], trans_cluster_max[1] - trans_cluster_min[1]))
        return trans_cluster_max[0] - trans_cluster_min[0], trans_cluster_max[1] - trans_cluster_min[1], cluster_max[2]

    @staticmethod
    def get_spin_theta(center_point):
        # 求将y的正半轴的中心点旋转到y的正半轴上，需要旋转的角度（用弧度表示）
        theta = 0
        if center_point[0] < 0:
            theta = -1*math.atan(center_point[1]/center_point[0])-math.pi/2
        if center_point[0] > 0:
            theta = math.pi/2-math.atan(center_point[1]/center_point[0])
        return theta

    @staticmethod
    def transfer_point(x, y, theta):
        # 将点逆时针旋转theta角度，返回旋转后的二维坐标
        t_x = math.cos(theta)*x-math.sin(theta)*y
        t_y = math.cos(theta)*y+math.sin(theta)*x
        return [t_x, t_y]


class RadarCluster:
    def __init__(self, eps, minpts, type, min_cluster_count, cluster_snr_limit, radar_index = 0):
        # 聚类参数：eps:核心点半径，minpts：成为核心点需要的最小点数，type:聚类维度
        self.eps = eps
        self.minpts = minpts
        self.type = type

        self.score_offset = cluster_common.score_offset
        self.divide_num = cluster_common.divide_num  # 可以分割出的人数

        # 成为人的聚类最低要求
        self.min_cluster_count = min_cluster_count
        self.cluster_snr_limit = cluster_snr_limit

        # 多帧点云融合
        self.mix_frame_num = cluster_common.mixed_num  # 前后1帧融合，3帧融合；前后2帧融合，5帧融合
        self.frame_data_list = []  # [{"frame_num":1,"points":[]},..]
        self.mixed_points = []

        # 结果，就是这一帧得到的person list
        self.frame_cluster_result = {'frame_num': 0, 'person_list': []}

        # 过去n帧的历史结果信息
        self.history_data_size = 2
        self.history_frame_cluster_result = []
        # 过去5帧的搜索区域
        self.search_dist = [0.35, 0.3]
        #self.search_dist = [0.5, 0.45, 0.4, 0.35, 0.3]

        self.radar_index = radar_index

    def do_cluster(self, frame_data):
        # 提取frame_data的帧号和点云数据
        self.frame_cluster_result['frame_num'] = frame_data['frame_num']
        #points = Cluster.frame_data_to_cluster_points(frame_data)
        points = []
        # for i in frame_data['point_list']:
        #     points += frame_data['point_list'][i]
        points = frame_data['point_list']
        #print(points)
        # 进行多帧点云融合，减少空帧，单帧无法得到好的点云的情况
        self.frame_cluster_result['frame_num'], points = self.mix_multi_frame_data(points, frame_data['frame_num'])

        self.mixed_points = points
        # print("帧号:", self.frame_cluster_result['frame_num'])
        # 加入点云显示队列


        # 将点云聚类得到初始聚类结果，UnidentifiedCluster
        unidentified_cluster_list = self.initial_cluster(points)

        # 对初始聚类结果进行分析，对满足人的要求的聚类转换为人，不满足的删除，对一个聚类包含多个人的进行分割
        person_list = self.trans_cluster_to_person(unidentified_cluster_list)

        # 结束这一帧的聚类分析，并保存结果

        self.save_result(person_list)

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
            unidentified_cluster_list.append(UnidentifiedCluster(cluster_tem_dict[i], self.mix_frame_num, self.history_data_size, self.radar_index))

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
            # if True:
            # print("过去有，不过滤")
                person_list.append(Person(unidentified_cluster))
                continue

            # 如果这个类 可以转换成一个人 将其转换
            if check_result == 1:
                person_list.append(Person(unidentified_cluster))

            # 如果这个类无法确定能转换成一个人，类比较大，对其尝试分割
            if check_result == 2:
                divided_cluster_list = self.divide_cluster(unidentified_cluster)
                # if len(divided_cluster_list) > 1:
                #     print("分割")
                for cluster in divided_cluster_list:
                    person_list.append(Person(cluster))
        return person_list

    def cluster_appear_time_in_latest_frame(self, cur_frame_cluster_list):
        # 根据最近几帧的聚类结果，对这一帧每一个聚类在过去出现的次数进行统计，并找出这个聚类能被每一帧分配几个人
        if len(self.history_frame_cluster_result) < self.history_data_size:
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
        if len(self.history_frame_cluster_result) > self.history_data_size:
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
        # 依次用高斯混合尝试这些分割，然后选择其中分数最高的k
        score = 0
        people_num = 1
        # print(test_k_set)
        # 简化操作  直接假定可以分成2,3,4个人
        tags = []
        for tem_k in self.divide_num:
            k_tags = GaussianMixture(n_components=tem_k).fit_predict(np_points)
            tem_score = self.get_points_score(unidentified_cluster.points, k_tags)
            # print("tem_score:%f" % tem_score)
            # except:
            #     continue
            if tem_score > score:
                tags[:] = k_tags
                score = tem_score
                people_num = tem_k
            else:
                break
        origin_score = Person.get_cluster_score2(unidentified_cluster)
        if score-self.score_offset < origin_score:  #0,75
            k = 1
        else:
            k = people_num

        # 当只有一个人时，直接返回
        if k == 1 or self.radar_index == 2:
            return [unidentified_cluster]
        print('I splitted!')
        return self.trans_points_to_unidentified_cluster_list(unidentified_cluster.points, tags)

    def get_points_score(self, points, tags):
        # 先对点分类
        tem_dict = self.cluster_by_tag(points, tags)
        all_score = 0
        for i in tem_dict:
            all_score += Person.get_cluster_score2(UnidentifiedCluster(tem_dict[i], self.mix_frame_num, self.history_data_size, self.radar_index))
        # 聚类得分
        # if metrics_score > 2000:
        #     metrics_score = 2000
        # cluster_score = metrics_score/2000
        # print("该聚类得分：%f" % cluster_score)
        # print("平均得分：%f" % (all_score/len(tem_dict)))
        return all_score/len(tem_dict)
