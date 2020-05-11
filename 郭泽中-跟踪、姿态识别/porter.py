from Kalman import Multi_Kalman_Tracker
import numpy as np
import json,math,threading,sys
import common
from visual import ApplicationWindow
from PyQt5 import QtWidgets

detection_range=8
M=common.M
G=0.6
min_in_last_times=30
min_out_last_times=60
rate=0.6
xmin=-detection_range/2*math.sqrt(3)
xmax=detection_range/2*math.sqrt(3)
ymax=detection_range

def analyze_data():
    filepath= 'data_5_10,1-7米双人对向走，中间有坐下，第1次/data_5_10,1-7米双人对向走，中间有坐下，第1次/cart_transfer_data.json'
    file=open(filepath)
    data=json.load(file)

    tracker=Multi_Kalman_Tracker(G,min_in_last_times,min_out_last_times,M,rate,xmin,xmax,ymax)

    kalman_heights=[]
    cluster_heights=[]
    all_postures=[0,0,0,0,0]

    for frame in data:
        point_list=data[frame]
        point_list=np.array(point_list)

        if len(point_list)!=0:
            clusters=point_list[:,:2]
            heights=point_list[:,-1]
        else:
            clusters=[]
            heights=[]

        # 临时加上的
        for i in range(len(heights)):
            heights[i] += 0

        cluster_heights.extend(heights)

        tracker.nextFrame(clusters,heights,int(frame))

        postures=tracker.get_each_person_posture()
        locations=tracker.get_each_person_location()
        #distances=tracker.get_each_person_distance()
        heights=tracker.get_each_person_height()
        #raw_heights=tracker.get_each_person_raw_height()
        assignment=tracker.get_assignment()

        common.loc_pos.put([locations,postures,tracker.get_cluster_num(),assignment,tracker.get_frame()])
        # common.loc_pos.put([locations,heights,tracker.get_cluster_num(),assignment,tracker.get_frame()])

        for id in heights:
            kalman_heights.append(heights[id])
            all_postures[postures[id]]+=1

    print(sum(all_postures))
    print([all_postures[i]/sum(all_postures) for i in range(1,5)])
    print('均值:',np.mean(kalman_heights),len(kalman_heights))
    print('方差:',np.std(kalman_heights))
    print('原始均值:',np.mean(cluster_heights),len(cluster_heights),'总帧数',len(data))
    print('原始方差:',np.std(cluster_heights))


if __name__=='__main__':
    thread=threading.Thread(target=analyze_data,args=())
    thread.start()
    # exit(1)
    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow(xmin, xmax, ymax)
    app.show()
    qapp.exec_()