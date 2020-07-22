import math
import numpy as np
import pandas as pd
from single_cluster.run_test import run_cluser

def polar_2_cart(polar_data,elev_delta,radar_height):
    xita=elev_delta/180*math.pi

    cart_data=dict()

    for frame in polar_data:
        point_list=polar_data[frame]['point_list']
        cart_data[frame]=dict()
        cart_data[frame]['frame_num']=polar_data[frame]['frame_num']
        cart_data[frame]['time_stamp']=polar_data[frame]['time_stamp']
        cart_point_list=[]
        cart_data[frame]['point_list']=cart_point_list

        for point in point_list:
            new_point=dict()

            range=point['azi']
            azi=point['elev']
            elev=point['range2']

            new_point['x']=range * math.cos(xita - elev) * math.sin(azi)
            new_point['y']=range * math.cos(xita - elev) * math.cos(azi)
            new_point['z']=radar_height-range * math.sin(xita - elev)
            new_point['pid']=point['pid']
            new_point['snr']=point['snr']
            new_point['doppler']=point['doppler']

            if point['doppler']!=0:
                cart_point_list.append(new_point)
    return cart_data

def get_distance_height_correlationship(cluster_data):
    distances = []
    heights = []
    for frame in cluster_data:
        point_list = np.array(cluster_data[frame])
        if len(point_list) == 0:
            continue
        highest_person = np.argmax(point_list[:, -1])
        distances.append(np.linalg.norm(point_list[highest_person][:2]))
        heights.append(point_list[highest_person][-1])

    mean_height=np.mean(heights)
    mean_distance=np.mean(distances)

    for i in range(len(heights)):
        heights[i]-=mean_height
        distances[i]-=mean_distance

    max_height=max(max(heights),abs(min(heights)))
    max_distance=max(max(distances),abs(min(distances)))

    for i in range(len(heights)):
        heights[i]/=max_height
        distances[i]/=max_distance

    heights = pd.Series(heights)
    distances = pd.Series(distances)
    corr_gust = heights.corr(distances)
    return corr_gust

def get_cluster_data(cart_data):
    return run_cluser(cart_data)

def cal_elev_delta_radar_height(polar_data,human_height):
    elev_delta=0
    step=32
    i=0
    radar_height=0

    cart_data = polar_2_cart(polar_data, elev_delta,radar_height)
    cluster_data = get_cluster_data(cart_data)
    correlationship = get_distance_height_correlationship(cluster_data)

    while i<100 and step>0.01:
        i+=1

        if abs(correlationship)<0.1:
            break
        elif correlationship>0:
            direction=1
        else:
            direction=-1

        new_elev_delta=elev_delta+step*direction
        new_cart_data=polar_2_cart(polar_data,new_elev_delta,radar_height)
        new_cluster_data=get_cluster_data(new_cart_data)
        new_correlationship=get_distance_height_correlationship(new_cluster_data)

        if abs(correlationship)<abs(new_correlationship):
            step/=2
        else:
            elev_delta=new_elev_delta
            correlationship=new_correlationship

    cart_data = polar_2_cart(polar_data, elev_delta,radar_height)
    cluster_data = get_cluster_data(cart_data)

    heights=[]
    for frame in cluster_data:
        point_list = cluster_data[frame]
        for point in point_list:
            heights.append(point[-1])

    RADAR_HEIGHT=human_height-np.mean(heights)+radar_height

    return elev_delta,RADAR_HEIGHT