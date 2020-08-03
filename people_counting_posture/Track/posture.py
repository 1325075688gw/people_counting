from Track import track_common
import numpy as np

class Posture():

    HEIGHT_THRESHOLD_BETWEEN_STAND_SIT=0.83
    HEIGHT_THRESHOLD_BETWEEN_SIT_LYING=0.65
    HEIGHT_THRESHOLD_BETWEEN_LYING_SIT=0.7

    MOVE_RANGE_THRESHOLD_BETWEEN_WALK_STAND=0.2
    VELOCITY_THRESHOLD_BETWEEN_WALK_STAND=0.9

    def __init__(self):
        self.postures=[1]
        self.last_posture=1

    def add_posture(self,height_rate,velocity,move_range):
        posture=self.cal_posture(height_rate,velocity,move_range)
        self.postures.append(posture)
        if len(self.postures)>track_common.MAX_SAVE_FRAMES:
            del self.postures[0]

    def get_posture(self):
        multiple = 1
        if self.last_posture == 3:
            multiple = 1.5

        if self.postures[-1]!=self.last_posture:
            length=min(len(self.postures),int(track_common.arg_smooth*track_common.frames_per_sec))
            counts=np.bincount(self.postures[-length:])
            most=np.argmax(counts)
            if counts[most]/length>track_common.posture_rate*multiple and most==self.postures[-1]:
                self.last_posture=self.postures[-1]
        return self.last_posture

    def cal_posture(self,height_rate,velocity,move_range):
        if velocity > self.VELOCITY_THRESHOLD_BETWEEN_WALK_STAND and move_range > self.MOVE_RANGE_THRESHOLD_BETWEEN_WALK_STAND:
            return 4
        if height_rate>self.HEIGHT_THRESHOLD_BETWEEN_STAND_SIT:
            return 1
        elif height_rate>self.HEIGHT_THRESHOLD_BETWEEN_SIT_LYING:
            if self.last_posture==3:
                if height_rate<=self.HEIGHT_THRESHOLD_BETWEEN_LYING_SIT:
                    return 3
            return 2
        else:
            return 3