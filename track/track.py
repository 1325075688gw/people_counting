import numpy as np
import copy

from origin_point_cloud import common
from track.height import Height
from track.posture import Posture
from track.utils import is_at_edge_out


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
        self.not_detected_times=0
        self.speed=[[0,0]]

    def get_last_speed(self):
        start=max(0,len(self.speed)-10)
        return np.array(self.speed)[start:].mean(axis=0)

    def get_last_location(self):
        return self.location

    def get_last_height(self):
        return self.height.get_last_height()

    def get_location(self):
        if len(self.locations)<common.delay_frames+1:
            return None
        if not common.doFilter:
            return self.locations[-common.delay_frames-1]

        radius=int(common.delay_frames*common.arg_smooth)
        start=max(-len(self.locations),-common.delay_frames-radius)
        end=min(-1,-common.delay_frames+radius+1)
        location=np.array(self.locations[start:end]).mean(axis=0)
        return location

    def get_posture(self):
        return self.posture.get_posture()

    def get_height(self):
        return self.height.get_height()

    def predict(self):
        speed=self.get_last_speed()
        self.predict_location=np.matmul(self.transitionMatrix,self.location)+np.matmul(self.B,speed)
        self.predict_NoiseCov=np.matmul(np.matmul(self.transitionMatrix,self.NoiseCov),self.transitionMatrix)+self.processNoiseCov
        print('能跑到这')
        self.height.predict()

    def update_posture(self):
        if len(self.locations) < common.delay_frames/2 + 2:
            return
        start = - int(common.delay_frames/2)
        points_for_cal = np.array(self.locations[start:])
        x = points_for_cal[:, 0]
        y = points_for_cal[:, 1]
        move_range = np.linalg.norm([np.std(x), np.std(y)])*2
        height_rate = self.height.get_height_rate()
        velocity = np.linalg.norm(self.speed[-int(common.delay_frames/2)- 1])

        self.posture.add_posture(height_rate, velocity, move_range)

    def not_detected_update(self,is_pre):
        location=self.get_last_location()
        speed=[0,0]

        self.locations.append(location)
        self.height.not_detected_update()
        self.speed.append(speed)

        self.update_posture()

        if is_pre or is_at_edge_out(location):
            self.not_detected_times+=1

    def update(self,point,height):
        location=self.get_last_location()

        #Kalman滤波更新
        K = np.matmul(self.predict_NoiseCov, np.linalg.inv(self.predict_NoiseCov + self.processNoiseCov))
        self.location = self.predict_location + np.matmul(K, point - self.predict_location)
        self.NoiseCov = np.matmul(np.mat(np.identity(2)) - K, self.predict_NoiseCov)

        if type(self.location)==np.matrix:
            self.location=self.location.A.ravel()
        #添加新位置、速度、身高
        speed=np.array(self.location)-np.array(location)

        self.locations.append(self.location)
        self.speed.append(speed)
        self.height.update(height)

        self.not_detected_times=max(0,self.not_detected_times-2)

        self.update_posture()