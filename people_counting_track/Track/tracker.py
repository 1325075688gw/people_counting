import numpy as np
import copy
from scipy.optimize import linear_sum_assignment

from Track.utils import is_out_area
from Track.track import Track
from Track import track_common

'''
    :NOTE:
        后端的运算是实时的，使用get_each_person_XXX方法获得的数据都是M帧前的，
        也就是说会有一个人为的M帧的延迟
'''

class Tracker():
    '''
        不会直接生成轨迹，而是先生成预轨迹，在满足一定条件后，将预轨迹转换为轨迹
    '''

    cluster_nums=[]
    origin_clusters=[]

    #初始化
    def __init__(self,min_accept_distance,longest_pause_frames,in_frames,out_frames,in_rate):
        self.tracks = dict()
        self.pre_tracks=dict()

        self.frame = 0  # 当前帧号
        self.max_id=1
        self.pre_max_id=1

        self.in_rate=in_rate
        self.out_frames=out_frames
        self.in_frames=in_frames
        self.longest_pause_frames=longest_pause_frames
        self.min_accept_distance=min_accept_distance    #同一条轨迹在相邻两帧之间最大的移动距离

    # 输入下一帧数据进行计算
    def nextFrame(self, clusters, heights):
        # 帧号+1
        self.frame += 1
        self.clusters, self.heights = self.preprocess_clusters(clusters, heights)

        self.origin_clusters.append(self.clusters)
        if len(self.origin_clusters)>track_common.MAX_SAVE_FRAMES:
            del self.origin_clusters[0]

        # 判断当前是否有轨迹存在
        self.predict()
        self.pre_predict()
        self.association()
        self.get_unused_clusters()
        self.pre_association()
        self.update_tracks()
        self.update_pre_tracks()
        self.deal_with_unassigned_points()
        self.validate_tracks()

        # 保存当前帧过滤后的聚类点
        self.cluster_nums.append(len(self.clusters))
        if len(self.cluster_nums)>track_common.MAX_SAVE_FRAMES:
            del self.cluster_nums[0]

    #根据给定位置与身高初始化预轨迹
    def init_pre_track(self,location,height):
        track=Track(location,height)
        self.pre_tracks[self.pre_max_id]=track
        self.pre_max_id+=1

    #预测每一条轨迹在下一帧中的位置
    def predict(self):
        for track_id in self.tracks:
            track=self.tracks[track_id]
            track.predict()

    #预测每一条预轨迹在下一帧中的位置
    def pre_predict(self):
        for track_id in self.pre_tracks:
            track=self.pre_tracks[track_id]
            track.predict()

    #使用匈牙利算法为轨迹分配点
    def association(self):
        self.assignment=dict()

        if len(self.clusters)==0 or len(self.tracks)==0:
            return

        distance=self.cal_distance()
        track_ids=list(self.tracks.keys())

        # row_ind, col_ind = self.get_assignment_hungary(distance)
        if len(self.clusters)==len(self.tracks):
            row_ind,col_ind=self.get_assignment_hungary(distance)
        else:
            row_ind,col_ind=self.get_assignment_min_distance(distance)

        for i in row_ind:
            j=col_ind[i]
            if j < len(self.clusters):
                track_id = track_ids[i]
                if distance[i][j] < self.min_accept_distance:
                    self.assignment[track_id] = j

    def get_unused_clusters(self):
        self.unused_clusters=[]
        self.unused_heights=[]
        for i in range(len(self.clusters)):
            if i not in self.assignment.values():
                self.unused_heights.append(self.heights[i])
                self.unused_clusters.append(self.clusters[i])

    #使用匈牙利算法为预轨迹分配点
    def pre_association(self):
        self.pre_assignment=dict()

        if len(self.unused_clusters)==0 or len(self.pre_tracks)==0:
            return

        distance=self.cal_pre_distance()
        track_ids=list(self.pre_tracks.keys())

        # row_ind, col_ind = self.get_assignment_hungary(distance)
        if len(self.unused_clusters)==len(self.pre_tracks):
            row_ind,col_ind=self.get_assignment_hungary(distance)
        else:
            row_ind,col_ind=self.get_assignment_min_distance(distance)

        for i in row_ind:
            j=col_ind[i]
            if j < len(self.unused_clusters):
                track_id=track_ids[i]
                if distance[i][j] < self.min_accept_distance:
                    self.pre_assignment[track_id] = j

    #更新轨迹
    def update_tracks(self):
        for track_id in self.tracks:
            track = self.tracks[track_id]
            if track_id in self.assignment.keys():
                cluster_index=self.assignment[track_id]
                track.update(self.clusters[cluster_index],self.heights[cluster_index])
            else:
                track.not_detected_update(False)

    #更新预轨迹
    def update_pre_tracks(self):
        for track_id in self.pre_tracks:
            track=self.pre_tracks[track_id]
            if track_id in self.pre_assignment.keys():
                cluster_index=self.pre_assignment[track_id]
                track.update(self.unused_clusters[cluster_index],self.unused_heights[cluster_index])
            else:
                track.not_detected_update(True)

    #处理未被分配的点
    def deal_with_unassigned_points(self):
        for i in range(len(self.unused_clusters)):
            if i not in self.pre_assignment.values():
                # if not is_at_edge_in(self.unused_clusters[i]):
                #     continue
                self.init_pre_track(self.unused_clusters[i],self.unused_heights[i])

    #验证轨迹合法性
    def validate_tracks(self):
        #正式轨迹
        to_be_deleted=[]
        for track_id in self.tracks:
            track=self.tracks[track_id]
            if track.not_detected_times>self.out_frames:
                to_be_deleted.append(track_id)
        for track_id in to_be_deleted:
            self.delete_track(track_id)
            # print(self.frame,'旧轨迹',track_id,'无了',len(self.tracks))

        #预轨迹
        to_be_deleted=[]
        for track_id in self.pre_tracks:
            track=self.pre_tracks[track_id]
            if len(track.locations)>=self.in_frames:
                to_be_deleted.append(track_id)
                if (self.in_frames-track.not_detected_times)/self.in_frames>self.in_rate:
                    self.pre2formal(track_id)
                    print(self.frame,'新轨迹',list(self.tracks.keys())[-1],track.not_detected_times,len(self.tracks))


        for track_id in to_be_deleted:
            self.delete_pre_track(track_id)

    #将预轨迹转换为轨迹
    def pre2formal(self,track_id):
        self.tracks[self.max_id]=self.pre_tracks[track_id]
        self.max_id+=1

    #删除指定轨迹
    def delete_track(self,track_id):
        del self.tracks[track_id]

    #删除指定预轨迹
    def delete_pre_track(self,track_id):
        del self.pre_tracks[track_id]

    #对输入聚类结果进行预处理
    def preprocess_clusters(self,clusters,heights):
        new_clusters=[]
        new_heights=[]

        for cluster,height in zip(clusters,heights):
            if not is_out_area(cluster):
                new_clusters.append(cluster)
                new_heights.append(height)

        return new_clusters,new_heights

    # 计算每条轨迹与当前帧中每个点之间的距离
    def cal_distance(self):
        # 计算得到每条轨迹的预测位置与聚类得到的点的距离
        distance = np.array([])
        for track_id in self.tracks:
            track = self.tracks[track_id]
            row = np.array([])
            for j in range(len(self.clusters)):
                row = np.append(row, [np.linalg.norm(self.clusters[j] - track.predict_location)*0.8+0.2*(track.height.predict_height-self.heights[j])], axis=0)
            if distance.size == 0:
                distance = np.array([row])
            else:
                distance = np.append(distance, [row], axis=0)

        return distance

    # 计算未被分配的点与每个预轨迹之间的距离
    def cal_pre_distance(self):
        # 计算得到每条轨迹的预测位置与聚类得到的点的距离
        distance = np.array([])
        for track_id in self.pre_tracks:
            track = self.pre_tracks[track_id]
            row = np.array([])
            for j in range(len(self.unused_clusters)):
                row = np.append(row, [np.linalg.norm(self.unused_clusters[j] - track.predict_location)*0.8+0.2*(track.height.predict_height-self.unused_heights[j])], axis=0)
            if distance.size == 0:
                distance = np.array([row])
            else:
                distance = np.append(distance, [row], axis=0)

        return distance

    # 使用匈牙利算法获得点与轨迹的匹配结果
    def get_assignment_hungary(self,distanceMatrix):
        row = len(distanceMatrix)
        col = len(distanceMatrix[0])
        matrix = copy.copy(distanceMatrix)

        while row != col:
            if row < col:
                sub_row = np.array([100.0] * col)
                matrix = np.append(matrix, [sub_row], axis=0)
                row += 1
            else:
                sub_col = np.array([100.0] * row)
                matrix = np.append(matrix.T, [sub_col], axis=0).T
                col += 1

        row_ind, col_ind = linear_sum_assignment(matrix)

        return row_ind, col_ind

    # 使用最短距离算法获得点与轨迹的匹配结果
    def get_assignment_min_distance(self,distanceMatrix):
        row_ind=[]
        col_ind=dict()

        if len(distanceMatrix)==0:
            return row_ind,col_ind

        while True:
            if len(col_ind)==len(distanceMatrix) or len(col_ind)==len(distanceMatrix[0]):
                break
            indexi=-1
            indexj=-1
            min_distance=float('inf')
            for i in range(len(distanceMatrix)):
                if i in col_ind:
                    continue
                for j in range(len(distanceMatrix[i])):
                    if j in col_ind.values():
                        continue
                    if distanceMatrix[i][j]<=min_distance:
                        min_distance=distanceMatrix[i][j]
                        indexi=i
                        indexj=j
            if indexi!=-1 and indexj!=-1:
                col_ind[indexi]=indexj
                row_ind.append(indexi)

        return row_ind,col_ind

    #获得每个人的身高
    def get_each_person_height(self):
        heights=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            height=track.get_height()
            if height is not None:
                heights[track_id]=round(height,2)

        return heights

    #获得每个人到雷达板的距离
    def get_each_person_distance(self):
        distances=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            distances[track_id]=np.linalg.norm(track.location)

        return distances

    #获得每个人的位置
    def get_each_person_location(self):
        locations=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            location=track.get_location()
            locations[track_id]=location

        return locations

    #获得每个人的姿态
    def get_each_person_posture(self):
        postures=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            posture=track.get_posture()
            postures[track_id]=posture

        return postures

    #获得每个人的原始身高
    def get_each_person_raw_height(self):
        raw_height=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            raw_height[track_id]=round(track.height.heights[-1],2)

        return raw_height

    #获得倒数第M+1帧的聚类个数
    def get_cluster_num(self):
        return self.cluster_nums[-1]

    def get_frame(self):
        return self.frame

    def get_origin_clusters(self):
        return self.origin_clusters[-1]