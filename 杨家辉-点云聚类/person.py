from height import Height
import math

class Person:
    person_length_upper_offset = 0.3
    person_length_lower_offset = -0.1
    # 长度方程
    # 训练数据 然后填进去
    person_length_coef = 0.115886
    person_length_intercept = 0.311095

    person_width_min = 0.255964
    person_width_max = 0.405964
    person_points = 125.214655
    # 训练结束

    coefficient_length = 0.4
    coefficient_width = 0.5
    coefficient_points = 0.1

    # 当人远离雷达，并在距离雷达boundary 外，不做过滤
    boundary = 6

    def __init__(self, unidentified_cluster):
        self.points = unidentified_cluster.points
        self.center_point = unidentified_cluster.center_point
        self.cur_height = self.compute_cluster_height()
        self.dist = unidentified_cluster.dist
        self.doppler_v = unidentified_cluster.doppler_v #靠近雷达  v为负值，  远离雷达 v为正值
        self.width = unidentified_cluster.width
        self.length = unidentified_cluster.length
        self.sum_snr = unidentified_cluster.sum_snr
        self.avg_snr = unidentified_cluster.avg_snr
        self.score = self.get_cluster_score2(unidentified_cluster)

    def compute_cluster_height(self):
        h = Height(1.5)
        cur_height = h.compute_height(self.points)
        return cur_height

    @staticmethod
    def check_person(unidentified_cluster, min_cluster_count, cluster_snr_limit):
        # 根据输入的点云聚类，判断点云: 不是人  返回0  一个人  返回1  多人 返回2

        # 当人远离雷达，在边界范围外时，不做噪声判断
        if not (unidentified_cluster.doppler_v > 0 and unidentified_cluster.dist > Person.boundary):
            # 点云snr和太小，或者点数太少，不是人
            if unidentified_cluster.sum_snr < cluster_snr_limit * unidentified_cluster.mix_frame_num:
                return 0
            if unidentified_cluster.points_num < min_cluster_count * unidentified_cluster.mix_frame_num:
                return 0

        if unidentified_cluster.length < 0.95 and unidentified_cluster.width < 0.55:
            return 1
        if unidentified_cluster.length < 0.6 and unidentified_cluster.width < 0.6:
            return 1

        return 2

    @staticmethod
    def get_cluster_score(unidentified_cluster):
        # 根据先验知识对点云打分，比较像一个人的点云分数较高，60，为基本满足一个人的点云
        score = 0
        # 长度在0.6~1.1之间，宽度在0.3~0.55之间，给80分
        if 0.7 < unidentified_cluster.length < 1 and 0.35 < unidentified_cluster.width < 0.5:
            return 80

        if 0.59 < unidentified_cluster.length < 1.1 and 0.25 < unidentified_cluster.width < 0.6:
            return 70

        if 0.5 < unidentified_cluster.length < 1.2 and 0.2 < unidentified_cluster.width < 0.6:
            return 60

        if 0.4 < unidentified_cluster.length < 1.3 and 0.2 < unidentified_cluster.width < 0.7:
            return 50

        if 0.4 < unidentified_cluster.length < 1.4 and 0.15 < unidentified_cluster.width < 0.75:
            return 40

        if unidentified_cluster.length > 1.5 or unidentified_cluster.width > 0.9:
            return 10

        return 25

    @staticmethod
    def get_cluster_score2(unidentified_cluster):
        # 宽度打分
        if unidentified_cluster.dist >= 2:
            if unidentified_cluster.width < Person.person_width_min:
                width_score = 1 - (Person.person_width_min - unidentified_cluster.width) / Person.person_width_min
            elif unidentified_cluster.width > Person.person_width_max:
                width_score = 1 - abs(unidentified_cluster.width - Person.person_width_max)/Person.person_width_max
            else:
                width_score = 1
        else:
            if unidentified_cluster.width < Person.person_width_min:
                width_score = 1 - (Person.person_width_min - unidentified_cluster.width) / Person.person_width_min
            elif unidentified_cluster.width > Person.person_width_max + 0.15:
                width_score = 1 - abs(unidentified_cluster.width - (Person.person_width_max + 0.15))/(Person.person_width_max+ 0.15)
            else:
                width_score = 1
        #print("原始宽度：%f，宽度得分：%f" % (unidentified_cluster.width, width_score))
        # 长度打分
        # 根据长度方程求出标准长度
        standard_length = Person.person_length_coef * unidentified_cluster.dist + Person.person_length_intercept
        person_length_min = standard_length + Person.person_length_lower_offset
        person_length_max = standard_length + Person.person_length_upper_offset
        if unidentified_cluster.length < standard_length + Person.person_length_lower_offset:
            length_score = 1 - (person_length_min - unidentified_cluster.length) / person_length_min
        elif unidentified_cluster.length > person_length_max:
            length_score = 1 - abs(unidentified_cluster.length - person_length_max)/person_length_max
        else:
            length_score = 1
        # print("长度得分：%f" % length_score)
        # 点数打分
        standard_points_score = Person.person_points
        points_num_score = 1 - abs(unidentified_cluster.points_num - standard_points_score)/standard_points_score
        #print("点数得分：%f" % points_num_score)

        score = Person.coefficient_length * length_score + Person.coefficient_width * width_score + \
                Person.coefficient_points * points_num_score
        # print("该人得分：%f" % score)
        return score
