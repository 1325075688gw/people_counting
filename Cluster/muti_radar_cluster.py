from Cluster.cluster import RadarCluster
import sys
sys.path.append('../')
from Cluster import cluster_common

class Cluster:
    def __init__(self, eps, minpts, type, min_cluster_count, cluster_snr_limit, radar_num=3):
        self.cluster_list = []
        self.radar_num = cluster_common.radar_num
        for i in range(self.radar_num):
            self.cluster_list.append(RadarCluster(eps=eps, minpts=minpts, type=type, min_cluster_count=min_cluster_count, cluster_snr_limit=cluster_snr_limit, radar_index = i))

        self.frame_cluster_result = {'frame_num': 0, 'person_list': []}

    def do_cluster(self, frame_data):
        radar_frame_data = {}
        for i in range(len(self.cluster_list)):
            radar_frame_data['frame_num'] = frame_data['frame_num']
            radar_frame_data['point_list'] = frame_data['point_list'][str(i)]
            self.cluster_list[i].do_cluster(radar_frame_data)

        self.update_frame_cluster_result()

    def get_height_list(self):
        height_dict = {}
        for i in range(len(self.cluster_list)):
            height_dict[i] = self.cluster_list[i].get_height_list()
        return height_dict

    def get_cluster_center_point_list(self):
        center_point_dict = {}
        for i in range(len(self.cluster_list)):
            center_point_dict[i] = self.cluster_list[i].get_cluster_center_point_list()
        return center_point_dict

    def update_frame_cluster_result(self):
        self.frame_cluster_result['person_list'] = []
        for i in range(len(self.cluster_list)):
            self.frame_cluster_result['frame_num'] = self.cluster_list[i].frame_cluster_result['frame_num']
            self.frame_cluster_result['person_list'] += self.cluster_list[i].frame_cluster_result['person_list']

    def put_points_show(self,point_cloud_show_queue):
        points = []
        for i in range(len(self.cluster_list)):
            points += self.cluster_list[i].mixed_points
        point_cloud_show_queue.put(points)
