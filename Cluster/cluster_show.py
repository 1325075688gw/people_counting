import sys
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
import time
import math

# sys.path.append("../") #for me
# sys.path.append("../src") #for me

from Origin_Point_Cloud.common import xmin,xmax,ymax,ymin,detection_range
from Cluster.point_cloud import PointCloud

class ClusterWindow():

    def __init__(self, x_range, y_range,point_cloud_show_queue,cluster_show_queue):
        self.app = QtGui.QApplication([])
        self.mw = QtGui.QMainWindow()
        self.mw.resize(x_range, y_range)
        self.view = pg.GraphicsLayoutWidget()  ## GraphicsView with GraphicsLayout inserted by default
        self.view.setBackground('w')
        self.mw.setCentralWidget(self.view)
        self.mw.show()
        self.mw.setWindowTitle('Show Cluster')

        self.point_cloud_show_queue=point_cloud_show_queue
        self.cluster_show_queue=cluster_show_queue

        #绘制聚类效果图
        self.cluster_plot = self.view.addPlot() #cluster_show
        self.cluster_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.cluster_plot.setRange(xRange=[xmin, xmax], yRange=[ymin, ymax], padding=0)
        self.cluster_color = ['#FFB6C1', '#FFD700', '#00FF7F', '#FFFF00', '#FFA500', '#597ADF', '#D72CFF',
            '#EDCE75', '#5BB1AB', '#844925', '#3FBED1', '#98352E', '#243FB5', '#3AF451', '#8AF979',
            '#EB5EFA', '#FDBB8D', '#6388B6', '#DB245D', '#3C9643', '#FCDC82', '#481B33', '#DAC9D4',
            '#F35595', '#2B6D76', '#228B22']
        self.frame_num_text = pg.TextItem()
        self.cluster_plot.addItem(self.frame_num_text)
        self.frame_num_text.setPos(-0.2, ymax)
        self.cluster_plot.addItem(self.cluster_scatter)
        self.snr_texts_item = []

        #绘制原始点云
        self.origin_plot = self.view.addPlot() #xoy
        self.origin_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.origin_plot.setRange(xRange=[xmin, xmax], yRange=[ymin, ymax], padding=0)
        self.origin_plot.addItem(self.frame_num_text)

        self.paintBorder()
        # self.paintLine()
        #绘制xoz yoz
        # self.view.nextRow()
        # self.plot1 = self.view.addPlot() #xoz
        # self.plot2 = self.view.addPlot() #yoz
        # self.scatter1 = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        # self.scatter2 = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        # self.plot1.setRange(xRange=[-4, 4], yRange=[-1, 2.5], padding=0)
        # self.plot2.setRange(xRange=[0, ymax], yRange=[-1, 2.5], padding=0)
        # self.plot1.addItem(self.scatter1)
        # self.plot2.addItem(self.scatter2)

    def paintBorder(self):
        angle = np.arange(math.pi / 6, math.pi * 5 / 6, math.pi / 60)
        x = np.cos(angle) * detection_range
        y = np.sin(angle) * detection_range
        x = np.insert(x, 0, 0)
        y = np.insert(y, 0, 0)
        x = np.insert(x, len(x), 0)
        y = np.insert(y, len(y), 0)
        self.origin_plot.plot(x, y)
        self.cluster_plot.plot(x, y)
        # self.origin_plot.plot(x,ymax-y)
        # self.cluster_plot.plot(x,ymax-y)

    def paintLine(self):
        self.origin_plot.plot([xmin,xmax],[ymin,ymax])
        self.cluster_plot.plot([xmin,xmax],[ymin,ymax])

    def update_data(self):

        #更新聚类数据
        point_cloud = []

        for text_item in self.snr_texts_item:
            text_item.setText("")
        if self.point_cloud_show_queue.empty() or self.cluster_show_queue.empty():
            return
        if not self.cluster_show_queue.empty():
            cluster_result = self.cluster_show_queue.get()
            person_list = cluster_result['person_list']
            points_spots = []
            people_spots = []
            for i in range(len(person_list)):
                if i < len(self.snr_texts_item):
                    self.snr_texts_item[i].setPos(person_list[i].center_point[0], person_list[i].center_point[1])
                    self.snr_texts_item[i].setText("v:%f,snr:%.2f,snr_sum:%d,height:%.2f" % (person_list[i].doppler_v, person_list[i].avg_snr, person_list[i].sum_snr, person_list[i].cur_height), color='#000000')
                else:
                    snr_text = pg.TextItem()
                    self.snr_texts_item.append(snr_text)
                    self.cluster_plot.addItem(snr_text)

                points_spots += [{'pos': np.array(point), 'brush': self.cluster_color[i]}
                                 for point in person_list[i].points]
                people_spots += [{'pos': np.array([person_list[i].center_point[0], person_list[i].center_point[1]]), 'size': 50,
                                  'brush': pg.mkBrush(255, 255, 255, 0),
                                  'pen': {'color': self.cluster_color[i], 'width': 2}}]
            self.cluster_scatter.setData(points_spots)
            self.cluster_scatter.addPoints(people_spots)
            # print(("第%d帧" % cluster_data['frame_num']))
            self.frame_num_text.setText(("第%d帧" % cluster_result['frame_num']), color='#000000')

        #更新原始点云数据，更新xoz  yoz
        point_cloud = []
        # if PointCloud.get_frame_pointcloud2(Origin_Point_Cloud, True):
        if PointCloud.get_frame_pointcloud3(point_cloud,self.point_cloud_show_queue):
            origin_spots = [{'pos': np.array(point)} for point in point_cloud]
            self.origin_scatter.setData(origin_spots)
            self.origin_plot.addItem(self.origin_scatter)


            # spots1 = [{'pos': np.array([point[0], point[2]])} for point in Origin_Point_Cloud]
            # spots2 = [{'pos': np.array([point[1], point[2]])} for point in point_cloud]
            # self.scatter2.setData(spots2)
            # self.scatter1.setData(spots1)
        #显示snr较大的点云
        # if PointCloud.get_frame_pointcloud2(Origin_Point_Cloud, True):
        #     Origin_Point_Cloud.sort(key=lambda x: x[4], reverse=True)
        #     end_index = math.ceil(len(Origin_Point_Cloud) * 0.3)
        #     origin_spots = [{'pos': np.array(point), 'brush': '#FFD700'} for point in Origin_Point_Cloud[:end_index]]
        #     origin_spots += [{'pos': np.array(point)} for point in Origin_Point_Cloud[end_index:]]
        #     self.origin_scatter.setData(origin_spots)
        #     self.origin_plot.addItem(self.origin_scatter)
        #
        #     spots1 = [{'pos': np.array(point)} for point in Origin_Point_Cloud]
        #     spots1 = [{'pos': np.array([point[0], point[2]]), 'brush': '#FFD700'} for point in Origin_Point_Cloud[:end_index]]
        #     spots1 += [{'pos': np.array([point[0], point[2]])} for point in Origin_Point_Cloud[end_index:]]
        #     spots2 = [{'pos': np.array([point[1], point[2]]), 'brush': '#FFD700'} for point in Origin_Point_Cloud[:end_index]]
        #     spots2 += [{'pos': np.array([point[1], point[2]])} for point in Origin_Point_Cloud[end_index:]]
        #     self.scatter2.setData(spots2)
        #     self.scatter1.setData(spots1)
        # print("队列长度？：", cluster_show_queue.qsize())
        # print("队列长度？：", point_cloud_show_queue.qsize())

    def run(self):
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update_data)  # 定时调用plotData函数
        timer.start(10)  # 多少ms调用一次
        QtGui.QApplication.instance().exec_()

def show_cluster(xrange,yrange,point_cloud_show_queue,cluster_show_queue):
    cw=ClusterWindow(xrange,yrange,point_cloud_show_queue,cluster_show_queue)
    cw.run()