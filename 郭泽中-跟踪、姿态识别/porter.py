from Kalman import Multi_Kalman_Tracker
import numpy as np
import json,math
from track_visual import visual
from matplotlib import pyplot as plt

plt.rcParams['font.family'] = ['sans-serif']
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus']=False

M=30
G=0.5
min_in_last_times=30
min_out_last_times=60
rate=0.4
xmin=-4*math.sqrt(3)
xmax=4*math.sqrt(3)
ymax=8

'''
test
'''
filepath= '在雷达照射方向上来回走动/data_4_22_1_1.6_在雷达照射方向上来回走动/cart_transfer_data.json'
file=open(filepath)
data=json.load(file)

tracker=Multi_Kalman_Tracker(G,min_in_last_times,min_out_last_times,M,rate,xmin,xmax,ymax)

fig=plt.figure(figsize=(xmax-xmin,ymax))
plt.ion()

all_clusters=[]
all_heights=[]
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

    all_clusters.append(clusters)
    tracker.nextFrame(clusters,heights,int(frame))

    postures=tracker.get_each_person_posture()
    locations=tracker.get_each_person_location()
    #distances=tracker.get_each_person_distance()
    heights=tracker.get_each_person_height()
    #raw_heights=tracker.get_each_person_raw_height()

    if len(all_clusters)>M:
        clusters=all_clusters[-M-1]
    else:
        clusters=[]

    for id in heights:
        all_heights.append(heights[id])
        all_postures[postures[id]]+=1

    visual(locations,postures,len(clusters))

print([all_postures[i]/sum(all_postures) for i in range(1,5)])
print('均值:',np.mean(all_heights))
print('方差:',np.std(all_heights))