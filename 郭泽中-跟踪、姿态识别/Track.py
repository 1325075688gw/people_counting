import numpy as np
from Height import Height
from posture import Posture

class Track():

    def __init__(self,id,s,P,frame,height):
        self.s_ = []
        self.P_ = []
        self.id=id
        self.s=s
        self.P=P
        self.first_frame=frame
        self.points=[s]
        self.u=np.array([[0,0]])
        self.height=Height(height)
        self.posture=Posture()

    def add_frame(self,position,speed,height):
        self.add_position(position)
        self.add_speed(speed)
        self.add_height(height)
        self.add_posture(self.height.get_height(),np.linalg.norm(speed))

    def add_position(self,position):
        self.points.append(position)
        if len(self.points)>100:
            del self.points[0]

    def add_speed(self,speed):
        self.u=np.append(self.u,[speed],axis=0)
        if len(self.u)>100:
            self.u=np.delete(self.u,0,axis=0)

    def add_height(self,height):
        self.height.add_height(height)

    def add_posture(self,height,velocity):
        posture=self.posture.cal_posture(height)
        self.posture.add_posture(posture)

    def get_posture(self,M,rate):
        if len(self.points)<M+1:
            return None
        x=np.array(self.points)[:-M-1,0]
        y=np.array(self.points)[:-M-1,1]
        if np.linalg.norm([np.std(x),np.std(y)])>0.5:
            return 4
        return self.posture.get_posture(M,rate)

    def get_location(self,M):
        if len(self.points)<M+1:
            return None
        return self.points[-M-1]