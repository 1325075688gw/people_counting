import numpy as np
import copy
from origin_point_cloud import common

MAX_SAVE_FRAMES = 150  # 用于限制内存占用

class Height():
    '''
        身高计算中有如下假设：
            1.人的身高在1.5m-2.1m
            2.在从一个人出现开始的头FRAME_NUMBER_FOR_HEIGHT_CAL帧中，人会出现站立的情况
            3.人在一开始进入雷达探测范围时是站着的
    '''

    FRAME_NUMBER_FOR_HEIGHT_CAL=100 #不可小于MAX_SAVE_FRAMES

    def __init__(self,height):
        self.height=[height]
        self.origin_height=[height]
        self.real_height=height
        self.index=1

        #kalman部分
        self.transitionMatrix = 1
        self.processNoiseCov = 0.01
        self.P = 0.01
        self.P_ = 0.01
        self.s = height
        self.s_ = 0

    def get_height_rate(self):
        return self.height[-1]/self.real_height

    def get_current_height(self,M):
        if len(self.height)<M+1:
            return None

        return self.height[-M-1]

    def predict(self):
        self.s_=self.s
        self.P_=self.P+self.processNoiseCov

    def update(self,height):
        K=self.P_/(self.P_+self.processNoiseCov)
        self.s=self.s_+K*(height-self.s_)
        self.P=(1-K)*self.P_

    def add_height(self,height):
        self.predict()
        self.update(height)

        self.height.append(self.s)
        self.origin_height.append(height)

        self.index+=1

        if len(self.height)==common.delay_frames:
            self.real_height=np.mean(self.height)

        if self.index%self.FRAME_NUMBER_FOR_HEIGHT_CAL==0 and self.index<=self.FRAME_NUMBER_FOR_HEIGHT_CAL*3:
            self.cal_real_height()

        if len(self.height)>MAX_SAVE_FRAMES:
            del self.height[0]
            del self.origin_height[0]

    def cal_real_height(self):
        min_block_length=7
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

        # mean_block_heights.append(np.mean(self.height[start:]))
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

        if len(self.postures)>MAX_SAVE_FRAMES:
            del self.postures[0]

    def get_posture(self,M,rate):
        if len(self.postures)<M/2+1:
            return None

        if self.postures[-int(M/2)-1]!=self.last_posture:
            counts=np.bincount(self.postures[-int(M/2):])
            most=np.argmax(counts)
            if counts[most]/(M/2)>rate and most==self.postures[-int(M/2)-1]:
                self.last_posture=self.postures[-int(M/2)-1]

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

class Track():

    def __init__(self,id,s,P,frame,height,M):
        self.s_ = []
        self.P_ = []
        self.id=id
        self.s=s
        self.P=copy.deepcopy(P)
        self.first_frame=frame
        self.points=[s]
        self.u=np.array([[0,0]])
        self.height=Height(height)
        self.posture=Posture()

        self.M=M

    def add_frame(self,position,speed,height):
        self.add_position(position)
        self.add_speed(speed)
        self.add_height(height)
        self.add_posture()

    def add_position(self,position):
        self.points.append(position)
        if len(self.points)>MAX_SAVE_FRAMES:
            del self.points[0]

    def add_speed(self,speed):
        self.u=np.append(self.u,[speed],axis=0)
        if len(self.u)>MAX_SAVE_FRAMES:
            self.u=np.delete(self.u,0,axis=0)

    def add_height(self,height):
        self.height.add_height(height)

    def add_posture(self):
        #根据当前帧的数据计算第-M/2-1帧的姿态
        if len(self.points)<self.M/2+1:
            return
        start=max(0,len(self.points)-self.M+1)
        points_for_cal=np.array(self.points[start:])
        x=points_for_cal[:,0]
        y=points_for_cal[:,1]
        move_range=np.linalg.norm([np.std(x),np.std(y)])
        height_rate=self.height.get_height_rate()
        velocity=np.linalg.norm(self.u[-int(self.M/2)-1])
        self.posture.add_posture(height_rate,velocity,move_range)

    def get_location(self,radius):
        if len(self.points)<self.M+1:
            return None
        # return self.points[-self.M-1]
        start=max(-len(self.points),int(-self.M*float(radius+1)/radius-1))
        end=int(-self.M*(radius-1)/radius+2)
        return np.array(self.points)[start:end].mean(axis=0)

    def get_height(self):
        return self.height.get_current_height(self.M)

    def get_posture(self,rate):
        return self.posture.get_posture(self.M,rate)