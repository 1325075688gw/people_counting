import os
import numpy as np
import json
import sklearn.cluster as skc


def read_and_cluster(filename):
    length_list = []
    width_list = []
    points_num_list = []
    #f = open(filename, 'r', encoding='utf-8')
    with open(filename, 'r', encoding='utf-8') as f:
        temfile = json.load(f)
        # print(temfile)
        for k in temfile:
            frame_data = temfile[k]

            #数据转换
            points = []
            for data in frame_data['point_list']:
                point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
                points.append(point)

            if points == []:
                continue
            #聚类
            X = np.array(points)
            X = X[:, :2]
            db = skc.DBSCAN(eps=0.25, min_samples=5).fit(X)
            tags = db.labels_

            #分类
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

            #滤波
            del_list = []
            for i in tem_dict:
                if i == -1 or len(tem_dict[i]) < 20:
                    del_list.append(i)

            for i in del_list:
                tem_dict.pop(i)

            #求长宽点数
            for i in tem_dict:
                cluster_max = np.max(tem_dict[i], axis=0)
                cluster_min = np.min(tem_dict[i], axis=0)
                length_list.append(cluster_max[0]-cluster_min[0])
                width_list.append(cluster_max[1]-cluster_min[1])
                points_num_list.append(len(tem_dict[i]))

    avg_length = np.mean(length_list)
    avg_width = np.mean(width_list)
    avg_points = np.mean(points_num_list)
    return [avg_length, avg_width, avg_points]


def run_training_data(base_path, path, filename, result_list):
    #print
    if filename == 'cart_transfer_data.json':
        #print(os.path.join(base_path, path, filename))
        result_list.append(read_and_cluster(os.path.join(base_path, path, filename)))
        #print(result_list)
        return
    try:
        next_file_names = os.listdir(os.path.join(base_path, path, filename))
        #print(next_file_names)
        for next_file_name in next_file_names:
            run_training_data(base_path, os.path.join(path, filename), next_file_name, result_list)
    except:
        pass
        #print(filename + "该文件不符合要求")

if __name__ == "__main__":
    base_path = '.\\training_data'  #不变
    path = ''  #调整这个参数
    filename = ''
    result_list = []
    run_training_data(base_path, path, filename, result_list)
    person_attr = np.mean(result_list, axis=0)
    print("person_length_max：%f,person_length_min：%f,person_width:%f,person_points:%f" % (person_attr[0]+0.3, person_attr[0]-0.15, person_attr[1], person_attr[2]))

