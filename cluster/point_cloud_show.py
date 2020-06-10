import sys
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
import math

import common

class ClusterWindow():

    def __init__(self, x_range, y_range,queue_for_cluster_transfer):
        self.app = QtGui.QApplication([])
        self.mw = QtGui.QMainWindow()
        self.mw.resize(x_range, y_range)
        self.view = pg.GraphicsLayoutWidget()  ## GraphicsView with GraphicsLayout inserted by default
        self.view.setBackground('w')
        self.mw.setCentralWidget(self.view)
        self.mw.show()
        self.mw.setWindowTitle('Show Point Cloud')

        self.queue_for_cluster_transfer=queue_for_cluster_transfer

        # 点云多帧融合
        self.mixed_num = 1
        self.points_list = []

        # 绘制原始点云
        self.origin_plot = self.view.addPlot() #xoy
        self.origin_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.origin_plot.setRange(xRange=[common.xmin,common.xmax], yRange=[0, common.ymax], padding=0)

        self.frame_num_text = pg.TextItem()
        self.origin_plot.addItem(self.frame_num_text)
        self.frame_num_text.setPos(-0.2, common.ymax)
        self.origin_plot.addItem(self.origin_scatter)

        self.paintBorder()

    def update_data(self):
        if self.queue_for_cluster_transfer.empty():
            return
        frame_data = self.queue_for_cluster_transfer.get()
        points = ClusterWindow.get_frame_point_cloud(frame_data)
        # 获取帧号
        frame_num = frame_data['frame_num'] - self.mixed_num//2
        self.points_list.append(points)
        if len(self.points_list) < self.mixed_num:
            return
        if len(self.points_list) > self.mixed_num:
            self.points_list.pop(0)
        point_cloud = []
        for each in self.points_list:
            point_cloud += each
        origin_spots = [{'pos': np.array(point)} for point in point_cloud]
        self.origin_scatter.setData(origin_spots)
        self.origin_plot.addItem(self.origin_scatter)
        self.frame_num_text.setText(("第%d帧" % frame_num), color='#000000')

    @staticmethod
    def get_frame_point_cloud(frame_data):
        return frame_data['point_list']

    def paintBorder(self):
        angle = np.arange(math.pi / 6, math.pi * 5 / 6, math.pi / 60)
        x = np.cos(angle) * common.ymax
        y = np.sin(angle) * common.ymax
        x = np.insert(x, 0, 0)
        y = np.insert(y, 0, 0)
        x = np.insert(x, len(x), 0)
        y = np.insert(y, len(y), 0)
        self.origin_plot.plot(x, y)

    def run(self):
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update_data)  # 定时调用plotData函数
        timer.start(10)  # 多少ms调用一次
        QtGui.QApplication.instance().exec_()

def point_cloud_show(xrange,yrange,queue_for_cluster_transfer):
    mw=ClusterWindow(xrange,yrange,queue_for_cluster_transfer)
    mw.run()