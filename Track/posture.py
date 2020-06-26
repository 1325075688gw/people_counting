from Origin_Point_Cloud import common
import numpy as np

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
        if len(self.postures)<common.delay_frames/2+1:
            return None

        if self.postures[-int(common.delay_frames/2)-1]!=self.last_posture:
            counts=np.bincount(self.postures[-int(common.delay_frames/2):])
            most=np.argmax(counts)
            if (counts[most]+1)/(common.delay_frames/2)>common.frame_rate and most==self.postures[-int(common.delay_frames/2)-1]:
                self.last_posture=self.postures[-int(common.delay_frames/2)-1]

        return self.last_posture

    def cal_posture(self,height_rate,velocity,move_range):
        if height_rate>self.HEIGHT_THRESHOLD_BETWEEN_STAND_SIT:
            if velocity > self.VELOCITY_THRESHOLD_BETWEEN_WALK_STAND or move_range > self.MOVE_RANGE_THRESHOLD_BETWEEN_WALK_STAND:
                return 4
            return 1
        elif height_rate>self.HEIGHT_THRESHOLD_BETWEEN_SIT_LYING:
            return 2
        else:
            return 3