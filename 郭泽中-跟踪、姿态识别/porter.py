from Kalman import Multi_Kalman_Tracker
import numpy as np
import json,math,threading,sys
import common
from visual import ApplicationWindow
from PyQt5 import QtWidgets

detection_range=8
M=30
G=0.9
min_in_last_times=30
min_out_last_times=60
rate=0.6
xmin=-detection_range/2*math.sqrt(3)
xmax=detection_range/2*math.sqrt(3)
ymax=detection_range

def analyze_data():
    filepath= 'data_5_4,1-7米随意走，2人第4次-v2/data_5_4,1-7米随意走，2人第4次/cart_transfer_data.json'
    file=open(filepath)
    data=json.load(file)

    tracker=Multi_Kalman_Tracker(G,min_in_last_times,min_out_last_times,M,rate,xmin,xmax,ymax)

    for frame in data:
        point_list=data[frame]
        point_list=np.array(point_list)

        if len(point_list)!=0:
            clusters=point_list[:,:2]
            heights=point_list[:,-1]
        else:
            clusters=[]
            heights=[]

        tracker.nextFrame(clusters,heights,int(frame))

        postures=tracker.get_each_person_posture()
        locations=tracker.get_each_person_location()

        common.loc_pos.put([locations,postures,tracker.get_frame()])


if __name__=='__main__':
    thread=threading.Thread(target=analyze_data,args=())
    thread.run()
    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow(xmin, xmax, ymax)
    app.show()
    qapp.exec_()