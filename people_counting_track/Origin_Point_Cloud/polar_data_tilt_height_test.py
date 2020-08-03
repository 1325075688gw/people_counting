import json
import math

from Origin_Point_Cloud.calculate_elev_delta_radar_height import get_cluster_data,get_distance_height_correlationship

def polar_2_cart(polar_data,elev_deltas,radar_heights):

    cart_data = [dict(),dict()]

    for frame in polar_data:
        for radar in polar_data[frame]:
            xita=elev_deltas[int(radar)]/180*math.pi
            radar_height=radar_heights[int(radar)]
            # radar=int(radar)
            point_list = polar_data[frame][radar]
            cart_data[int(radar)][frame] = dict()
            cart_point_list = []
            cart_data[int(radar)][frame]['point_list'] = cart_point_list
            cart_data[int(radar)][frame]['frame_num']=frame

            for point in point_list:
                new_point = dict()

                range = point['range2']
                azi = point['azi']
                elev = point['elev']

                new_point['x'] = range * math.cos(xita - elev) * math.sin(azi)
                new_point['y'] = range * math.cos(xita - elev) * math.cos(azi)
                new_point['z'] = radar_height - range * math.sin(xita - elev)
                new_point['snr'] = point['snr']
                new_point['doppler'] = point['doppler']

                if point['doppler'] != 0:
                    cart_point_list.append(new_point)

    return cart_data

def read_data():
    file=open('./data/data_7_28，ODS6m,1人，两块板子，第1次/polar_data1.json')
    data=json.load(file)
    return data

def run():
    tilts=[36,24]
    heights=[2.84,2.42]

    polar_data=read_data()

    cart_data=polar_2_cart(polar_data,tilts,heights)
    for data in cart_data:
        cluster_data=get_cluster_data(data)
        correlationship=get_distance_height_correlationship(cluster_data)
        print(correlationship)

if __name__=='__main__':
    run()