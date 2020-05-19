from tracker import Tracker
import numpy as np
import json,math,threading,sys
import common
from visual import VisualModule
from PyQt5 import QtWidgets

min_accept_distance=0.5
longest_pause_frames=500
out_frames=30
in_frames=60
in_rate=0.5

tracker=Tracker(min_accept_distance,longest_pause_frames,out_frames,in_frames,in_rate)

def analyze_data():
    filepath= 'data/5.15_data/data_5_15,距离雷达3米处双人左右并排行走。第1次雷达2.15米，11度/cart_transfer_data.json'
    file=open(filepath)
    data=json.load(file)

    for frame in data:
        point_list=data[frame]
        point_list=np.array(point_list)

        if len(point_list)!=0:
            clusters=point_list[:,:2]
            heights=point_list[:,-1]
        else:
            clusters=[]
            heights=[]

        tracker.nextFrame(clusters,heights)

        postures=tracker.get_each_person_posture()
        locations=tracker.get_each_person_location()

        common.loc_pos.put([locations,postures])


if __name__=='__main__':
    thread=threading.Thread(target=analyze_data,args=())
    thread.run()
    qapp = QtWidgets.QApplication(sys.argv)
    app = VisualModule()
    app.show()
    qapp.exec_()