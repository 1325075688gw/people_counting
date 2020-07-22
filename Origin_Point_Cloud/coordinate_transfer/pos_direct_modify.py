import math
import random
import numpy as np
import json
import sklearn.cluster as skc

def read_data(filepath):
    filepath1=filepath+'/0/cart_data.json'
    filepath2=filepath+'/1/cart_data.json'
    file1=open(filepath1)
    file2=open(filepath2)
    data1=json.load(file1)
    data2=json.load(file2)
    return data1,data2

def cal_radar2_pos_direct(pos1,dy1,filepath):
    dy1=np.array(dy1)
    dy1=dy1/np.linalg.norm(dy1)

    data1,data2=read_data(filepath)
    center_points1,center_points2=get_center_points(data1,data2)
    dy2=cal_radar2_direct(pos1,dy1,center_points1,center_points2)
    pos2=cal_radar2_pos(pos1,dy1,dy2,center_points1,center_points2)
    return dy2,pos2

def cal_radar2_pos(pos1,dy1,dy2,center_points1,center_points2):
    dx2=[dy2[1],-dy2[0]]

    radar2_positions=[]
    for i in range(len(center_points1)):
        point1=center_points1[i]
        point2=center_points2[i]

        point1=cart_transfer(pos1,dy1,point1)
        point1=np.array(point1)

        point=point1-np.array(point2).dot(np.array([dx2,dy2]))

        radar2_positions.append(point)

    radar2_positions=np.array(radar2_positions)
    return np.mean(radar2_positions,axis=0)

#根据所有帧数据计算得到雷达2在世界坐标系下的方向向量
def cal_radar2_direct(pos1,dy1,center_points1,center_points2):
    vectors=[]
    for i in range(len(center_points1)):
        for j in range(i+1,len(center_points1)):

            point1=center_points1[i]
            point2=center_points2[i]
            point3=center_points1[j]
            point4=center_points2[j]

            point1=cart_transfer(pos1,dy1,point1)
            point3=cart_transfer(pos1,dy1,point3)

            vector=cal_vector(point1,point2,point3,point4)
            vectors.append(vector/np.linalg.norm(vector))

    vectors=np.array(vectors)
    mean_vector=np.mean(vectors,axis=0)
    return mean_vector/np.linalg.norm(mean_vector)

def cal_vector(point1,point2,point3,point4):
    matrix=np.array([[point4[1]-point2[1],point4[0]-point2[0]],
                     [point2[0]-point4[0],point4[1]-point2[1]]])
    return np.linalg.inv(matrix).dot(np.array([point3[0]-point1[0],point3[1]-point1[1]]))

def cart_transfer(pos1,dy1,point):
    dx1=np.array([dy1[1],-dy1[0]])
    return np.array(pos1)+np.array(point).dot(np.array([dx1,dy1]))

def get_center_points(data1,data2):
    keys1=list(data1.keys())
    keys2=list(data2.keys())

    center_points1=[]
    center_points2=[]

    for i in range(min(len(keys1),len(keys2))):
        points1=data1[keys1[i]]['point_list']
        points2=data2[keys2[i]]['point_list']
        center_point1=get_center_point(points1)
        center_point2=get_center_point(points2)
        if center_point1!=[] and center_point2!=[]:
            center_points1.append(center_point1)
            center_points2.append(center_point2)

    return center_points1,center_points2

def get_center_point(points):
    if len(points) == 0:
        return []
    # 聚类
    X = np.array(points)
    X = X[:, :2]
    db = skc.DBSCAN(eps=0.25, min_samples=10).fit(X)
    tags = db.labels_

    # 分类
    tem_dict = {}
    # 按照类别将点分类
    key_list = []
    for i in tags:
        if i not in key_list:
            key_list.append(i)
    for i in key_list:
        tem_dict[i] = []
    for t, point in zip(tags, points):
        tem_dict[t].append(point)

    # 滤波
    del_list = []
    for i in tem_dict:
        if i == -1 or len(tem_dict[i]) < 20:
            del_list.append(i)

    for i in del_list:
        tem_dict.pop(i)

    # 如果有多个聚类结果 取点数最多的作为代表
    points_tem_max = 0
    index = -1
    for i in tem_dict:
        if len(tem_dict) > points_tem_max:
            points_tem_max = len(tem_dict)
            index = i

    center = []
    if points_tem_max > 0:
        cluster_center_point = np.mean(tem_dict[index], axis=0)
        center = [cluster_center_point[0], cluster_center_point[1]]
    return center

def cal_radar2_params(x,y,dx,dy,filepath):
    pos1=[x,y]
    dy1=[dx,dy]

    return cal_radar2_pos_direct(pos1,dy1,filepath)

if __name__=='__main__':
    pass