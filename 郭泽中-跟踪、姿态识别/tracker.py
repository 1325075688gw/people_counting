import copy,common,math
import numpy as np
from scipy.optimize import linear_sum_assignment

class Posture():

    HEIGHT_THRESHOLD_BETWEEN_STAND_SIT=0.85
    HEIGHT_THRESHOLD_BETWEEN_SIT_LYING=0.58

    MOVE_RANGE_THRESHOLD_BETWEEN_WALK_STAND=0.25
    VELOCITY_THRESHOLD_BETWEEN_WALK_STAND=0.016

    def __init__(self):
        self.postures=[1]
        self.last_posture=1

    def add_posture(self,height_rate,velocity,move_range):
        posture=self.cal_posture(height_rate,velocity,move_range)
        self.postures.append(posture)
        if len(self.postures)>common.MAX_SAVE_FRAMES:
            del self.postures[0]

    def get_posture(self):
        if len(self.postures)<common.delay_frames+1:
            return None

        if self.postures[-common.delay_frames-1]!=self.last_posture:
            counts=np.bincount(self.postures[-common.delay_frames:])
            most=np.argmax(counts)
            if counts[most]/common.delay_frames>common.frame_rate and most==self.postures[-common.delay_frames-1]:
                self.last_posture=self.postures[-common.delay_frames-1]

        return self.last_posture

    def cal_posture(self,height_rate,velocity,move_range):
        if velocity > self.VELOCITY_THRESHOLD_BETWEEN_WALK_STAND or move_range > self.MOVE_RANGE_THRESHOLD_BETWEEN_WALK_STAND:
            return 4
        if height_rate>self.HEIGHT_THRESHOLD_BETWEEN_STAND_SIT:
            return 1
        elif height_rate>self.HEIGHT_THRESHOLD_BETWEEN_SIT_LYING:
            return 2
        else:
            return 3

class Height():

    FRAME_NUMBER_FOR_HEIGHT_CAL=100

    def __init__(self,height):
        self.height=[height]
        self.real_height=height
        self.index=1

        self.transitionMatrix = 1
        self.processNoiseCov = 0.03
        self.P = 0.03
        self.P_ = 0.03
        self.s = height
        self.s_ = 0

    def get_last_height_rate(self):
        return self.height[-1]/self.real_height

    def get_last_height(self):
        return self.height[-1]

    def get_current_height(self):
        if len(self.height)<common.delay_frames+1:
            return None

        return self.height[-common.delay_frames-1]

    def get_current_height_rate(self):
        if len(self.height)<common.delay_frames+1:
            return None

        return self.height[-common.delay_frames-1]/self.real_height

    def predict(self):
        self.s_=self.s
        self.P_=self.P+self.processNoiseCov

    def not_detected_update(self):
        self.height.append(self.height[-1])

    def update(self,height):
        self.predict()
        K=self.P_/(self.P_+self.processNoiseCov)
        self.s=self.s_+K*(height-self.s_)
        self.P=(1-K)*self.P_

        self.height.append(self.s)
        self.index += 1

        if len(self.height) == common.delay_frames:
            self.real_height = np.mean(self.height)

        if self.index % self.FRAME_NUMBER_FOR_HEIGHT_CAL == 0 and self.index <= self.FRAME_NUMBER_FOR_HEIGHT_CAL * 5:
            self.cal_real_height()

        if len(self.height) > common.MAX_SAVE_FRAMES:
            del self.height[0]

    def cal_real_height(self):
        min_block_length=10
        sub_to_cut=0.1

        i=min_block_length
        start=0
        mean_block_heights=[]
        height_blocks=[]

        while i<len(self.height)-min_block_length:
            mean1=np.mean(self.height[i-min_block_length:i])
            mean2=np.mean(self.height[i:i+min_block_length])
            if abs(mean1-mean2)>=sub_to_cut:
                # mean_block_heights.append(np.mean(self.height[start:i]))
                height_blocks.append(self.height[start:i])
                start=i
            i+=min_block_length

        height_blocks.append(self.height[start:])

        std=0.01
        while std<0.1:
            for height_block in height_blocks:
                if np.std(height_block)<std:
                    mean_block_heights.append(np.mean(height_block))
            if len(mean_block_heights)>0:
                break
            std+=0.01

        if len(mean_block_heights)==0:
            mean_block_heights.append(np.mean(self.height))
            return

        height=np.max(mean_block_heights)

        if height>self.real_height:
            self.real_height=height*0.2+self.real_height*0.8
        else:
            self.real_height=height*0.05+self.real_height*0.95

        print('real_height:',self.real_height)

class Track():

    transitionMatrix = np.array([[1, 0], [0, 1]])  # 状态转移矩阵
    processNoiseCov = np.array([[1, 0], [0, 1]]) * 0.03  # 过程噪声矩阵
    B = np.array([[1, 0], [0, 1]])  # B

    def __init__(self,location,height):
        self.locations=[location]
        self.height=Height(height)
        self.posture=Posture()
        self.location=location
        self.NoiseCov=copy.deepcopy(self.processNoiseCov)
        self.is_pre=True
        self.not_detected_times=0
        self.speed=[[0,0]]

    def get_last_speed(self):
        return self.speed[-1]

    def get_last_location(self):
        return self.location

    def get_last_height(self):
        return self.height.get_last_height()

    def get_location(self):
        if len(self.locations)<common.delay_frames+1:
            return None
        radius=int(common.delay_frames*common.arg_smooth)
        start=max(-len(self.locations),-common.delay_frames-radius)
        end=min(-1,-common.delay_frames+radius+1)
        location=np.array(self.locations[start:end]).mean(axis=0)
        return location

    def get_posture(self):
        return self.posture.get_posture()

    def predict(self):
        speed=self.get_last_speed()
        self.predict_location=np.matmul(self.transitionMatrix,self.location)+np.matmul(self.B,speed)
        self.predict_NoiseCov=np.matmul(np.matmul(self.transitionMatrix,self.NoiseCov),self.transitionMatrix)+self.processNoiseCov

    def update_posture(self):
        if len(self.locations) < common.delay_frames + 1:
            return
        start = max(0, len(self.locations) - common.delay_frames + 1)
        points_for_cal = np.array(self.locations[start:])
        x = points_for_cal[:, 0]
        y = points_for_cal[:, 1]
        move_range = np.linalg.norm([np.std(x), np.std(y)])*2
        height_rate = self.height.get_current_height()
        velocity = np.linalg.norm(self.speed[-common.delay_frames- 1])
        self.posture.add_posture(height_rate, velocity, move_range)

    def not_detected_update(self):
        location=self.get_last_location()
        speed=[0,0]

        self.locations.append(location)
        self.height.not_detected_update()
        self.speed.append(speed)

        self.update_posture()

    def update(self,point,height):
        #Kalman滤波更新
        K = np.matmul(self.predict_NoiseCov, np.linalg.inv(self.predict_NoiseCov + self.processNoiseCov))
        self.location = self.predict_location + np.matmul(K, point - self.predict_location)
        self.NoiseCov = np.matmul(np.mat(np.identity(2)) - K, self.predict_NoiseCov)
        #应该是因为numpy的矩阵向量相乘问题，有时会出现[[]]，有时不会
        # if len(self.location)==1:
        #     self.location=self.location[0]
        if type(self.location)==np.matrix:
            self.location=self.location.A.ravel()
        #添加新位置、速度、身高
        location=self.get_last_location()
        speed=np.array(self.location)-np.array(location)
        self.locations.append(self.location)
        self.speed.append(speed)
        self.height.update(height)

        self.not_detected_times=max(0,self.not_detected_times-2)

        self.update_posture()

class Tracker():

    def __init__(self,min_accept_distance,longest_pause_frames,out_frames,in_frames,in_rate):
        self.frame_num=0
        self.max_id=1
        self.tracks=dict()
        self.points=[]
        self.heights=[]

        self.min_accept_distance=min_accept_distance
        self.longest_pause_frames=longest_pause_frames
        self.out_frames=out_frames
        self.in_frames=in_frames
        self.in_rate=in_rate

    def nextFrame(self,points,heights):
        self.points=points
        self.heights=heights

        self.predict()
        self.assign_points_to_tracks()

        self.deal_unassigned_points()
        self.update_unassigned_tracks()
        self.update_assigned_tracks()

        self.validate_tracks()

    def predict(self):
        for track_id in self.tracks:
            track=self.tracks[track_id]
            track.predict()

    def get_tracks_locations(self,tracks):
        locations=[]
        for track_id in tracks:
            track=tracks[track_id]
            locations.append(track.get_last_location())
        return locations

    def get_tracks_heights(self,tracks):
        heights=[]
        for track_id in tracks:
            track=tracks[track_id]
            heights.append(track.get_last_height())
        return heights

    def assign_points_to_tracks(self):
        self.assignment=dict()

        if len(self.points)==0 or len(self.tracks)==0:
            return

        locations=self.get_tracks_locations(self.tracks)
        heights=self.get_tracks_heights(self.tracks)
        track_ids=list(self.tracks)

        distanceMatrix=self.cal_distances(locations,self.points,heights,self.heights)

        if len(self.tracks)==len(self.points):
            assigned_tracks,assignment=self.get_assignment_hungary(distanceMatrix)
        else:
            assigned_tracks,assignment=self.get_assignment_min_distance(distanceMatrix)

        for assigned_track in assigned_tracks:
            point_index=assignment[assigned_track]
            if point_index>=len(self.points):
                continue
            if distanceMatrix[assigned_track][point_index]>self.min_accept_distance:
                continue
            self.assignment[track_ids[assigned_track]]=point_index

    def get_assignment_min_distance(self,distanceMatrix):
        assigned_tracks=[]
        assignment = dict()

        if len(distanceMatrix) == 0:
            return assignment

        while True:
            if len(assignment) == len(distanceMatrix) or len(assignment) == len(distanceMatrix[0]):
                break
            indexi = -1
            indexj = -1
            min_distance = float('inf')
            for i in range(len(distanceMatrix)):
                if i in assignment:
                    continue
                for j in range(len(distanceMatrix[i])):
                    if j in assignment.values():
                        continue
                    if distanceMatrix[i][j] <= min_distance:
                        min_distance = distanceMatrix[i][j]
                        indexi = i
                        indexj = j
            if indexi != -1 and indexj != -1:
                assignment[indexi] = indexj
                assigned_tracks.append(indexi)

        return assigned_tracks,assignment

    def get_assignment_hungary(self,distanceMatrix):
        assigned_tracks, assigment = linear_sum_assignment(distanceMatrix)
        return assigned_tracks, assigment
    # def get_assignment_hungary(self,distanceMatrix):
    #     row = len(distanceMatrix)
    #     col = len(distanceMatrix[0])
    #     matrix = copy.copy(distanceMatrix)
    #
    #     while row != col:
    #         if row < col:
    #             sub_row = np.array([100.0] * col)
    #             matrix = np.append(matrix, [sub_row], axis=0)
    #             row += 1
    #         else:
    #             sub_col = np.array([100.0] * row)
    #             matrix = np.append(matrix.T, [sub_col], axis=0).T
    #             col += 1
    #
    #     assigned_tracks, assigment = linear_sum_assignment(matrix)
    #
    #     return assigned_tracks, assigment

    def cal_distances(self,locations,new_locations,heights,new_heights):
        distance = np.array([])
        for location,height in zip(locations,heights):
            row = np.array([])
            for new_location,new_height in zip(new_locations,new_heights):
                row = np.append(row, [np.linalg.norm(location - new_location) * 0.8 + 0.2 * (
                            height - new_height)], axis=0)
            if distance.size == 0:
                distance = np.array([row])
            else:
                distance = np.append(distance, [row], axis=0)

        return distance

    def deal_unassigned_points(self):
        for i in range(len(self.heights)):
            if i not in self.assignment.values():
                if self.frame_num>self.longest_pause_frames:
                    if self.is_at_edge(self.points[i]):
                        self.init_track(self.points[i],self.heights[i])
                else:
                    self.init_track(self.points[i],self.heights[i])

    def init_track(self,location,height):
        track=Track(location,height)
        self.tracks[self.max_id]=track
        self.max_id+=1

    def update_unassigned_tracks(self):
        for track_id in self.tracks:
            if track_id not in self.assignment.keys():
                track=self.tracks[track_id]
                track.not_detected_update()

                if self.is_at_edge(track.get_last_location()):
                    track.not_detected_times+=1

    def del_track(self,track_id):
        del self.tracks[track_id]

    def update_assigned_tracks(self):
        for track_id in self.tracks:
            if track_id not in self.assignment:
                continue
            point_index=self.assignment[track_id]
            self.tracks[track_id].update(self.points[point_index],self.heights[point_index])

    def validate_tracks(self):
        to_be_deleted=[]
        for track_id in self.tracks:
            track=self.tracks[track_id]
            if track.not_detected_times==self.out_frames:
                to_be_deleted.append(track_id)
            if track.is_pre:
                if len(track.locations)==self.in_frames:
                    if track.not_detected_times/self.in_frames>self.in_rate:
                        if track_id not in to_be_deleted:
                            to_be_deleted.append(track_id)
                    else:
                        track.is_pre=False

        for track_id in to_be_deleted:
            self.del_track(track_id)

    def is_at_edge(self,location):
        x=location[0]
        y=location[1]

        if x<common.xmin+0.5 or x>common.xmax-0.5 or y<common.ymin or y>common.ymax-0.5:
            return True

        if abs(x)>y*math.sqrt(3)-0.5:
            return True

        return False

    def get_each_person_location(self):
        locations=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            if track.is_pre:
                continue
            location=track.get_location()
            if location is not None:
                locations[track_id]=location
        return locations

    def get_each_person_posture(self):
        postures=dict()

        for track_id in self.tracks:
            track=self.tracks[track_id]
            if track.is_pre:
                continue
            posture=track.get_posture()
            if posture is not None:
                postures[track_id]=posture
        return postures