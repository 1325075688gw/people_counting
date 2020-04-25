import numpy as np
import matplotlib.pyplot as plt
import sys
import common
sys.path.append(r"../龚伟-点云检测")


def get_frame_pointcloud(framedata):
    pointcloud = []
    for data in framedata['point_list']:
        point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
        pointcloud.append(point)
    return pointcloud


def show2d_xoy(pointcloud):
    # ax = fig.add_subplot(111)
    X = np.array(pointcloud)
    if X != []:
        plt.scatter(X[:, 0], X[:, 1], marker='.')
    plt.scatter(0, 0, c='g')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim(-4, 4)
    plt.ylim(0, 8)
    #ax.legend(loc='upper left')


def show2d_xoz(pointcloud, fig):
    ax = fig.add_subplot(122)
    X = np.array(pointcloud)
    if X != []:
        ax.scatter(X[:, 0], X[:, 2], marker='.')
    ax.scatter(0, 0, c='g')
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    ax.set_xlim(-4, 4)
    ax.set_ylim(-1, 2)
    #ax.legend(loc='upper left')


def doppler_filter(pointcloud):
    res = []
    for point in pointcloud:
        if point[3] != 0:
            res.append(point)
    return res



def show_pointcloud(framedata):
    pointcloud = get_frame_pointcloud(framedata)
    # pointcloud = doppler_filter(pointcloud)
    show2d_xoy(pointcloud)
    # show2d_xoz(pointcloud, fig)

def run_show_pointcloud():
    fig = plt.figure(figsize=(10, 10))
    num = 0
    while 1:
        num += 1
        frame_data = common.queue_for_cluster_transfer.get()
        frame_num = frame_data["frame_num"]
        if num < 4:
            continue
        else:
            num = 0
            show_pointcloud(frame_data)
            plt.title("frame_num:{0}".format(frame_num))
            plt.pause(0.0001)
            plt.clf()
        #print(len(cluster_show['cluster']))
        #if cluster_show['frame_num'] >865:
        #	plt.pause(15)


# if __name__ == "__main__":
#     run_show_pointcloud()