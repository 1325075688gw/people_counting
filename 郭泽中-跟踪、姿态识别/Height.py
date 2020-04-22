import numpy as np

'''
    算法思想：
        设N=7
        对一个人，其前N帧不显示，从N+1帧开始显示
        从第N+1帧开始，计算其左侧N帧平均高度和其右侧N帧平均高度，取相差较小侧加上当前帧计算平均身高作为当前帧实际身高
        也就是说，对一个人，其前N帧不显示，且显示还有N帧延迟
'''

class Height():

    def __init__(self,height):
        self.height=[height]
        self.origin_height=[height]

        #kalman部分
        self.transitionMatrix = 1
        self.processNoiseCov = 0.01
        self.P = 0.01
        self.P_ = 0.01
        self.s = height
        self.s_ = 0

    def get_height(self):
        return self.height[-1]

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
        self.origin_height.append(self.s)

        self.predict()
        self.update(height)

        self.height.append(self.s)

        if len(self.height)>100:
            del self.height[0]