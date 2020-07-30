import sys
sys.path.append(r"../") #for me
from Origin_Point_Cloud import common #for me

class PointCloud():
    def __init__(self):
        pass

    @staticmethod
    def framedata_to_pointcloud(frame_data):
        pointcloud = []
        for data in frame_data['point_list']:
            point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
            pointcloud.append(point)
        return pointcloud

    @staticmethod
    def doppler_filter(pointcloud):
        res = []
        for point in pointcloud:
            if point[3] != 0:
                res.append(point)
        return res

#for qt_show
    @staticmethod
    def get_frame_pointcloud(pointcloud, dofilter):
        if common.queue_for_cluster_transfer.empty():
            return False
        frame_data = common.queue_for_cluster_transfer.get()
        pointcloud = PointCloud.framedata_to_pointcloud(frame_data)
        if dofilter:
            pointcloud = PointCloud.doppler_filter(pointcloud)
        return True


### for cluster_show

    @staticmethod
    def get_frame_pointcloud2(pointcloud, dofilter):
        if common.point_cloud_show_queue.empty():
            return False
        frame_data = common.point_cloud_show_queue.get()
        pointcloud[:] = PointCloud.framedata_to_pointcloud(frame_data)
        if dofilter:
            pointcloud[:] = PointCloud.doppler_filter(pointcloud)
        #print(pointcloud[0])
        return True

    @staticmethod
    def get_frame_pointcloud3(pointcloud,point_cloud_show_queue):
        if point_cloud_show_queue.empty():
            return False
        pointcloud[:] = point_cloud_show_queue.get()
        return True