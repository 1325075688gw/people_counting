import numpy as np
import copy

from Origin_Point_Cloud import common
from Track import track_common
from Track.height import Height
from Track.posture import Posture
from Track.utils import is_at_edge_out


class Track():

    transitionMatrix = np.array([[1, 0,1,0], [0, 1,0,1],[0,0,1,0],[0,0,0,1]])  # 状态转移矩阵
    processNoiseCov = np.array([[1, 0,0,0], [0, 1,0,0],[0,0,1,0],[0,0,0,1]]) * 0.03  # 过程噪声矩阵
    B = np.array([[0,0], [0, 0],[1,0],[0,1]])  # B

    def __init__(self,location,height):
        self.current_status = [location[0],location[1],0,0]
        self.status=[self.current_status]
        self.height=Height(height)
        self.posture=Posture()
        self.NoiseCov=copy.deepcopy(self.processNoiseCov)
        self.not_detected_times=0
        self.acc=[[0,0]]
        self.last_point=location

    def get_last_speed(self):
        start=max(0,len(self.status)-10)
        return np.array(self.status)[start:,2:].mean(axis=0)

    def get_last_speed_all(self):
        return np.array(self.status)[:,2:].mean(axis=0)

    def get_predict_location(self):
        return self.predict_status[:2]

    def get_last_location(self):
        return self.location

    def get_last_height(self):
        return self.height.get_last_height()

    def get_location(self):
        if not track_common.doFilter:
            return self.location

        start=max(-len(self.status),-int(track_common.frames_per_sec*track_common.arg_smooth))
        location=np.array(self.status)[start:,:2].mean(axis=0)
        return location

    def get_posture(self):
        return self.posture.get_posture()

    def get_height(self):
        return self.height.get_height()

    def predict(self):
        acc=self.acc[-1]

        self.predict_status=np.matmul(self.transitionMatrix,self.current_status)+np.matmul(self.B,acc)
        self.predict_NoiseCov=np.matmul(np.matmul(self.transitionMatrix,self.NoiseCov),self.transitionMatrix)+self.processNoiseCov
        self.height.predict()

    def update_posture(self):
        start=max(-len(self.status),-int(track_common.frames_per_sec*track_common.arg_smooth))
        points_for_cal = np.array(self.status)[start:,:2]
        x = points_for_cal[:, 0]
        y = points_for_cal[:, 1]
        move_range = np.linalg.norm([np.std(x), np.std(y)])*2
        height_rate = self.height.get_height_rate()
        velocity = np.mean([np.linalg.norm(np.array(self.status)[i,2:]) for i in range(start,len(self.status))])*track_common.frames_per_sec

        self.posture.add_posture(height_rate, velocity, move_range)

    def not_detected_update(self,is_pre):
        # self.current_status=self.predict_status
        # acc=[self.current_status[i]-self.status[-1][i] for i in range(2,4)]
        acc=[0,0]

        self.acc.append(acc)
        self.status.append(self.current_status)
        self.height.not_detected_update()

        self.update_posture()

        if is_pre or is_at_edge_out(self.current_status[:2]):
            self.not_detected_times+=1
        else:
            self.not_detected_times+=0.5

        if len(self.status)>track_common.MAX_SAVE_FRAMES:
            del self.status[0]
            del self.acc[0]

    def update(self,point,height):
        speed=np.array(point)-np.array(self.last_point)
        status=np.array([point[0],point[1],speed[0],speed[1]])
        self.last_point=point

        #Kalman滤波更新
        K = np.matmul(self.predict_NoiseCov, np.linalg.inv(self.predict_NoiseCov + self.processNoiseCov))
        self.current_status=self.predict_status+np.matmul(K,status-self.predict_status)
        self.NoiseCov = np.matmul(np.mat(np.identity(4)) - K, self.predict_NoiseCov)

        if type(self.current_status)==np.matrix:
            self.current_status=self.current_status.A.ravel()
        #添加新位置、速度、身高
        acc=np.array(self.current_status[2:])-np.array(self.status[-1][2:])

        self.status.append(self.current_status)
        self.acc.append(acc)
        self.height.update(height)

        self.not_detected_times=max(0,self.not_detected_times-1)

        self.update_posture()

        if len(self.status)>track_common.MAX_SAVE_FRAMES:
            del self.status[0]
            del self.acc[0]