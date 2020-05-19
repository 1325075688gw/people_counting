from height import Height


class Person:
    person_length = 0.4
    person_width = 0.15

    def __init__(self, unidentified_cluster):
        self.points = unidentified_cluster.points
        self.center_point = unidentified_cluster.center_point
        self.cur_height = self.compute_cluster_height()
        self.doppler_v = unidentified_cluster.doppler_v
        self.width = unidentified_cluster.width
        self.length = unidentified_cluster.length
        self.sum_snr = unidentified_cluster.sum_snr
        self.avg_snr = unidentified_cluster.avg_snr

    def compute_cluster_height(self):
        h = Height(1.5)
        cur_height = h.compute_height(self.points)
        return cur_height

    @staticmethod
    def check_person(unidentified_cluster, min_cluster_count, cluster_snr_limit):
        # 根据输入的点云聚类，判断点云: 不是人  返回0  一个人  返回1  多人 返回2

        # 点云snr和太小，或者点数太少，不是人
        if unidentified_cluster.sum_snr < cluster_snr_limit * unidentified_cluster.mix_frame_num:
            return 0
        if unidentified_cluster.points_num < min_cluster_count * unidentified_cluster.mix_frame_num:
            return 0

        if unidentified_cluster.length < 0.8 and unidentified_cluster.width < 0.5:
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
