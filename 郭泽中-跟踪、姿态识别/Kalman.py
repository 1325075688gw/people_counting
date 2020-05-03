import numpy as np
from Track import Track
import math,copy
from scipy.optimize import linear_sum_assignment

'''
    NOTE:
        后端的运算是实时的，使用get_each_person_XXX方法获得的数据都是M帧前的，
        也就是说会有一个人为的M帧的延迟
'''

class Multi_Kalman_Tracker():
    '''
        不会直接生成轨迹，而是先生成预轨迹，在满足一定条件后，将预轨迹转换为轨迹
    '''

    cluster_nums=[]

    #初始化
    def __init__(self,G,min_in_last_times,min_out_last_times,M,rate,xmin,xmax,ymax):
        self.measurementMatrix=np.array([[1, 0,], [0, 1]])    #H
        self.transitionMatrix=np.array([[1, 0], [0, 1]])  #状态转移矩阵
        self.processNoiseCov=np.array([[1, 0], [0, 1]]) * 0.03    #过程噪声矩阵
        self.B=np.array([[10, 0], [0, 1]])    #B
        self.d = dict()  # 存储当前帧中为轨迹分配的类
        self.tracks = dict()  # 所有轨迹
        self.pre_tracks=dict()
        self.clusters = []  # 当前帧中的点
        self.not_detected_times=dict()
        self.not_move_times=dict()
        self.pre_not_detected_times=dict()
        self.heights=[]    #当前帧中的每个人的身高
        self.unused_clusters=[]
        self.unused_heights=[]

        self.frame = 0  # 当前帧号
        self.successive_times = 0  # 用于判断是否生成新轨迹
        self.plength = 2
        self.max_id=1
        self.pre_max_id=1

        self.M=M    #NOTE:M必须是偶数
        self.rate=rate
        self.min_in_last_times=min_in_last_times
        self.min_out_last_times=min_out_last_times
        self.G=G    #同一条轨迹在相邻两帧之间最大的移动距离
        self.xmin=xmin  #空间最小横向坐标
        self.xmax=xmax  #空间最大横向坐标
        self.ymax=ymax  #空间的最大纵向长度

    '''
        在不存在任何预轨迹的情况下调用此函数
    '''
    def init_pre_tracks(self):
        for i in range(len(self.unused_clusters)):
            self.init_pre_track(self.unused_clusters[i],self.unused_heights[i])

    #根据给定位置与身高初始化预轨迹
    def init_pre_track(self,s,height):
        track=Track(self.pre_max_id,s,self.processNoiseCov,self.frame,height,self.M)
        self.pre_tracks[self.pre_max_id]=track
        self.pre_not_detected_times[self.pre_max_id]=0
        self.pre_max_id+=1

    #预测每一条轨迹在下一帧中的位置
    def predict(self):
        for track_id in self.tracks:
            track=self.tracks[track_id]
            start=max(0,len(track.u)-self.min_in_last_times)
            track.s_=np.matmul(self.transitionMatrix,track.s)+np.matmul(self.B,track.u[start:].mean(axis=0))
            track.P_=np.matmul(np.matmul(self.transitionMatrix,track.P),self.transitionMatrix.T)+self.processNoiseCov

    #预测每一条预轨迹在下一帧中的位置
    def pre_predict(self):
        for track_id in self.pre_tracks:
            track=self.pre_tracks[track_id]
            start=max(0,len(track.u)-self.min_in_last_times)
            track.s_=np.matmul(self.transitionMatrix,track.s)+np.matmul(self.B,track.u[start:].mean(axis=0))
            track.P_=np.matmul(np.matmul(self.transitionMatrix,track.P),self.transitionMatrix.T)+self.processNoiseCov

    #使用匈牙利算法为轨迹分配点
    def association(self):

        if len(self.clusters)==0 or len(self.tracks)==0:
            return

        distance=self.cal_distance()

        #使用匈牙利算法为每条轨迹分配一个点
        ind=self.assign_point_to_track_min_distance(distance)
        track_ids=list(self.tracks.keys())

        for i in ind:
            if distance[i][ind[i]]<self.G:
                self.d[track_ids[i]]=ind[i]

    #使用匈牙利算法为预轨迹分配点
    def pre_association(self):

        if len(self.unused_clusters)==0 or len(self.pre_tracks)==0:
            return

        distance=self.cal_pre_distance()

        #使用匈牙利算法为每条轨迹分配一个点
        ind=self.assign_point_to_track_min_distance(distance)
        track_ids=list(self.pre_tracks.keys())

        for i in ind:
            if distance[i][ind[i]]<self.G:
                self.d[track_ids[i]]=ind[i]

    def update(self):
        self.unused_clusters=[]
        self.unused_heights=[]

        self.deal_unassigned_tracks()
        self.update_assigned_tracks()

        self.d=dict()

        if len(self.pre_tracks)==0:
            self.init_pre_tracks()
        else:
            self.pre_predict()
            self.pre_association()
            self.deal_unassigned_pre_tracks()
            self.update_assigned_pre_tracks()

    #更新未被分配到点的轨迹
    def update_unassigned_track(self,track_id):
        track=self.tracks[track_id]

        track.add_frame(track.s,[0,0],track.height.s)

    #更新未被分配到点的预轨迹
    def update_unassigned_pre_track(self,track_id):
        track=self.pre_tracks[track_id]

        track.add_frame(track.s,[0,0],track.height.s)

    #处理未被分配到点的轨迹
    def deal_unassigned_tracks(self):
        to_be_deleted=[]
        for track_id in self.tracks:
            track=self.tracks[track_id]
            #若轨迹未被分配到点
            if track_id not in self.d:
                #若上一帧中轨迹位于边缘
                if self.is_at_edge(track.s):
                    self.not_detected_times[track_id] += 0.5
                    if self.not_detected_times[track_id]>self.min_out_last_times:
                        to_be_deleted.append(track_id)
                    else:
                        self.update_unassigned_track(track_id)
                else:
                    self.not_move_times[track_id]+=1
                    if self.not_detected_times[track_id]>self.min_out_last_times*1.5:
                        to_be_deleted.append(track_id)
                    else:
                        self.update_unassigned_track(track_id)
            else:
                self.not_detected_times[track_id]=max(0,self.not_detected_times[track_id]-1)
                self.not_move_times[track_id]=max(0,self.not_move_times[track_id]-1.5)

        for track_id in to_be_deleted:
            self.delete_track(track_id)

    #处理未被分配到点的预轨迹
    def deal_unassigned_pre_tracks(self):
        to_be_deleted=[]
        for track_id in self.pre_tracks:
            track=self.pre_tracks[track_id]
            #若轨迹未被分配到点
            if track_id not in self.d:
                self.pre_not_detected_times[track_id] += 1
                self.update_unassigned_pre_track(track_id)

                if self.frame - track.first_frame > self.min_in_last_times:
                    to_be_deleted.append(track_id)
                    if (self.min_in_last_times - self.pre_not_detected_times[
                        track_id]) / self.min_in_last_times > self.rate:
                        self.tracks[self.max_id] = self.pre_tracks[track_id]
                        self.not_detected_times[self.max_id] = 0
                        self.not_move_times[self.max_id]=0
                        self.max_id += 1

                        print('---------------------------')
                        print('生成新轨迹！',self.frame)
                        print(self.not_move_times)
                        print(self.pre_not_detected_times)

        for track_id in to_be_deleted:
            self.delete_pre_track(track_id)

    #删除指定轨迹
    def delete_track(self,track_id):
        del self.tracks[track_id]
        del self.not_detected_times[track_id]

    #删除指定预轨迹
    def delete_pre_track(self,track_id):
        del self.pre_tracks[track_id]
        del self.pre_not_detected_times[track_id]

    #更新被分配到点的轨迹
    def update_assigned_tracks(self):

        used_clusters=[]

        for track_id in self.tracks:
            track=self.tracks[track_id]

            #若当前轨迹没有被分配到点，则不对当前点进行更新
            try:
                cluster_id=self.d[track_id]
            except:
                continue

            #将该点标记为已被分配
            used_clusters.append(cluster_id)

            K=np.matmul(track.P_,np.linalg.inv(track.P_+self.processNoiseCov))
            track.s=track.s_+np.matmul(K,self.clusters[cluster_id]-track.s_)
            track.P=np.matmul(np.mat(np.identity(self.plength))-K,track.P_)

            #解决numpy中矩阵与向量相乘导致向量本来应该是x，变成[x]
            if len(track.s)==1:
                track.s=np.array(track.s)[0]

            # 为当前点计算速度(只考虑xy方向的)
            track.add_frame(track.s,track.s-track.points[-1],self.heights[cluster_id])
        #track_update end

        #处理当前帧未被处理的点

        for j in range(len(self.clusters)):
            if j not in used_clusters:
                self.unused_heights.append(self.heights[j])
                self.unused_clusters.append(self.clusters[j])

    #更新被分配到点的预轨迹
    def update_assigned_pre_tracks(self):

        used_clusters=[]
        to_be_deleted=[]

        for track_id in self.pre_tracks:
            track=self.pre_tracks[track_id]

            #若当前轨迹没有被分配到点，则不对当前点进行更新
            try:
                cluster_id=self.d[track_id]
            except:
                continue

            #将该点标记为已被分配
            used_clusters.append(cluster_id)

            K=np.matmul(track.P_,np.linalg.inv(track.P_+self.processNoiseCov))
            track.s=track.s_+np.matmul(K,self.unused_clusters[cluster_id]-track.s_)
            track.P=np.matmul(np.mat(np.identity(self.plength))-K,track.P_)

            #解决numpy中矩阵与向量相乘导致向量本来应该是x，变成[x]
            if len(track.s)==1:
                track.s=np.array(track.s)[0]

            # 为当前点计算速度(只考虑xy方向的)
            track.add_frame(track.s,track.s-track.points[-1],self.unused_heights[cluster_id])

            if self.frame - track.first_frame > self.min_in_last_times:
                to_be_deleted.append(track_id)
                if (self.min_in_last_times - self.pre_not_detected_times[
                    track_id]) / self.min_in_last_times > self.rate:
                    self.tracks[self.max_id] = self.pre_tracks[track_id]
                    self.not_detected_times[self.max_id] = 0
                    self.not_move_times[self.max_id]=0
                    self.max_id += 1

                    print('---------------------------')
                    print('生成新轨迹！', self.frame)
                    print(self.not_move_times)
                    print(self.pre_not_detected_times)

        for track_id in to_be_deleted:
            self.delete_pre_track(track_id)
        #track_update end

        #处理当前帧未被处理的点
        for j in range(len(self.unused_clusters)):
            if j in used_clusters:
                continue
            if self.is_at_edge(self.unused_clusters[j]):
                self.init_pre_track(self.unused_clusters[j],self.unused_heights[j])

    #输入下一帧数据进行计算
    def nextFrame(self,clusters,heights,frame):
        #将上一帧的数据清空
        self.d=dict()
        self.deleted_tracks=[]
        #帧号+1
        self.frame+=1
        self.clusters,self.heights=self.preprocess_clusters(clusters,heights)
        #判断当前是否有轨迹存在
        self.predict()
        self.association()
        self.update()

        #保存当前帧过滤后的聚类点
        self.cluster_nums.append(len(self.clusters))

    #判断轨迹是否位于边缘
    def is_at_edge(self,s):
        #判断是否位于给定空间边缘
        if s[0]<self.xmin+0.5 or s[0]>self.xmax-0.5 or s[1]>self.ymax-0.5:
            return True
        #判断是否位于雷达探测范围边缘
        if s[1]<1:
            return True
        if abs(s[0])>s[1]*math.sqrt(3)-0.5:
            return True
        return False

    #判断点是否已经超出了给定范围
    def is_out_area(self,s):
        # 判断是否位于给定空间之外
        if s[0] < self.xmin or s[0] > self.xmax or s[1] > self.ymax:
            return True
        # 判断是否位于雷达探测范围之外
        if s[1] < 0:
            return True
        if abs(s[0]) > s[1] +0.2:
            return True
        return False

    #对输入聚类结果进行预处理
    def preprocess_clusters(self,clusters,heights):
        new_clusters=[]
        new_heights=[]

        for cluster,height in zip(clusters,heights):
            if not self.is_out_area(cluster):
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
                row = np.append(row, [np.linalg.norm(self.clusters[j] - track.s_)], axis=0)
            if distance.size == 0:
                distance = np.array([row])
            else:
                distance = np.append(distance, [row], axis=0)

        return distance

    def cal_pre_distance(self):
        # 计算得到每条轨迹的预测位置与聚类得到的点的距离
        distance = np.array([])
        for track_id in self.pre_tracks:
            track = self.pre_tracks[track_id]
            row = np.array([])
            for j in range(len(self.unused_clusters)):
                row = np.append(row, [np.linalg.norm(self.unused_clusters[j] - track.s_)], axis=0)
            if distance.size == 0:
                distance = np.array([row])
            else:
                distance = np.append(distance, [row], axis=0)

        return distance

    def assign_point_to_track(self,distanceMatrix):
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

    def assign_point_to_track_min_distance(self,distanceMatrix):
        ind=dict()

        if len(distanceMatrix)==0:
            return ind

        while True:
            if len(ind)==len(distanceMatrix) or len(ind)==len(distanceMatrix[0]):
                break
            indexi=-1
            indexj=-1
            min_distance=float('inf')
            for i in range(len(distanceMatrix)):
                if i in ind:
                    continue
                for j in range(len(distanceMatrix[i])):
                    if j in ind.values():
                        continue
                    if distanceMatrix[i][j]<=min_distance:
                        min_distance=distanceMatrix[i][j]
                        indexi=i
                        indexj=j
            if indexi!=-1 and indexj!=-1:
                ind[indexi]=indexj

        return ind

    #获得每个人的身高
    def get_each_person_height(self):
        heights=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            height=track.get_height()
            if height is not None:
                heights[track_id]=round(height,2)

        return heights

    #获得每个人的速度
    def get_each_person_velocity(self):
        velocity=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            velocity[track_id]=np.linalg.norm(track.u[-1])

        return velocity

    #获得每个人到雷达板的距离
    def get_each_person_distance(self):
        distances=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            if len(track.points)>=self.M+1:
                distances[track_id]=np.linalg.norm(track.points[-1-self.M])

        return distances

    #获得每个人的位置
    def get_each_person_location(self):
        locations=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            location=track.get_location(6)
            if location is not None:
                locations[track_id]=location

        return locations

    def get_each_person_current_frame_location(self):
        locations=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            location=track.points[-1]
            locations[track_id]=location

        return locations

    #获得每个人的姿态
    def get_each_person_posture(self):
        postures=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            posture=track.get_posture(self.rate)
            if posture is not None:
                postures[track_id]=posture

        return postures

    #获得每个人的原始身高
    def get_each_person_raw_height(self):
        raw_height=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            if len(track.height.origin_height)>self.M:
                raw_height[track_id]=round(track.height.origin_height[-1-self.M],2)

        return raw_height

    #获得倒数第M+1帧的聚类个数
    def get_cluster_num(self):
        if len(self.cluster_nums)<self.M+1:
            return 0
        else:
            return self.cluster_nums[-self.M-1]