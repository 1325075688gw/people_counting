from Origin_Point_Cloud import common
import numpy as np

class Height():

    FRAME_NUMBER_FOR_HEIGHT_CAL=100

    def __init__(self,height):
        self.heights=[height]
        self.real_height=height
        self.index=1

        self.transitionMatrix = 1
        self.processNoiseCov = 0.03
        self.noiseCov = 0.03
        self.predict_NoiseCov = 0.03
        self.height = height
        self.predict_height = 0

    def get_last_height_rate(self):
        return self.heights[-1]/self.real_height

    def get_last_height(self):
        return self.heights[-1]

    def get_height(self):
        if len(self.heights)<common.delay_frames+1:
            return None

        return self.heights[-common.delay_frames-1]

    def get_height_rate(self):
        return self.heights[-int(common.delay_frames/2)-1]/self.real_height

    def predict(self):
        self.predict_height=self.height
        self.predict_NoiseCov=self.noiseCov+self.processNoiseCov

    def not_detected_update(self):
        self.heights.append(self.heights[-1])

    def update(self,height):
        K=self.predict_NoiseCov/(self.predict_NoiseCov+self.processNoiseCov)
        self.height=self.predict_height+K*(height-self.predict_height)
        self.noiseCov=(1-K)*self.predict_NoiseCov

        self.heights.append(self.height)
        self.index += 1

        if len(self.heights) == common.delay_frames:
            self.real_height = np.mean(self.heights)

        if self.index % self.FRAME_NUMBER_FOR_HEIGHT_CAL == 0 and self.index <= self.FRAME_NUMBER_FOR_HEIGHT_CAL * 5:
            self.cal_real_height()

        if len(self.heights) > common.MAX_SAVE_FRAMES:
            del self.heights[0]

    def cal_real_height(self):
        min_block_length=10
        sub_to_cut=0.1

        i=min_block_length
        start=0
        mean_block_heights=[]
        height_blocks=[]

        while i<len(self.heights)-min_block_length:
            mean1=np.mean(self.heights[i-min_block_length:i])
            mean2=np.mean(self.heights[i:i+min_block_length])
            if abs(mean1-mean2)>=sub_to_cut:
                height_blocks.append(self.heights[start:i])
                start=i
            i+=min_block_length

        height_blocks.append(self.heights[start:])

        std=0.01
        while std<0.1:
            for height_block in height_blocks:
                if np.std(height_block)<std:
                    mean_block_heights.append(np.mean(height_block))
            if len(mean_block_heights)>0:
                break
            std+=0.01

        if len(mean_block_heights)==0:
            mean_block_heights.append(np.mean(self.heights))
            return

        height=np.max(mean_block_heights)

        if height>self.real_height:
            self.real_height=height*0.2+self.real_height*0.8
        else:
            self.real_height=height*0.05+self.real_height*0.95