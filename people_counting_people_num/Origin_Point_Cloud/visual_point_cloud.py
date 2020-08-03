import numpy as np
import math,sys
from PyQt5.QtWidgets import QGridLayout,QWidget
import pyqtgraph as pg
from PyQt5 import QtWidgets

from Cluster.point_cloud import PointCloud
from Origin_Point_Cloud.common import azi_range

class FourPlots(QWidget):

    def __init__(self,point_cloud_show_queue,xmin,xmax,ymax,detection_range):
        super().__init__()

        self.point_cloud_show_queue=point_cloud_show_queue
        self.xmin=xmin
        self.ymin=0
        self.xmax=xmax
        self.ymax=ymax
        self.detection_range=detection_range

        self.people=dict()

        self.initUI()
        self.timer_start()

    def initUI(self):

        self.setGeometry(50,50,950*(self.xmax-self.xmin)/self.ymax,950)
        self.layout = QGridLayout(self)
        self.setLayout(self.layout)
        #初始化子图
        self.point_plt=pg.PlotWidget()
        #设置子图坐标范围
        self.point_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[self.ymin,self.ymax],padding=0)
        #将子图添加到主窗口
        self.layout.addWidget(self.point_plt,0,0)
        #设置子图背景
        self.point_plt.setBackground('w')
        #初始化子图显示数据
        self.origin_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        #将子图显示数据与子图绑定
        self.point_plt.addItem(self.origin_scatter)

        #画边界
        # self.paintBorder(self.point_plt)

    def paintBorder(self, plt):

        angle = np.arange(math.pi/2-azi_range, math.pi * (1/2+1/20), math.pi / 600)
        x = np.cos(angle) * self.detection_range
        y = np.sin(angle) * self.detection_range

        angle=math.pi*(1/2+1/20)

        x = np.insert(x, 0, 0)
        y = np.insert(y, 0, 0)

        x = np.insert(x, len(x), math.cos(angle)/math.sin(angle)*3)
        y = np.insert(y, len(y), 3)

        x = np.insert(x, len(x), -math.tan(azi_range)*3)
        y = np.insert(y, len(y), 3)

        x = np.insert(x, len(x), 0)
        y = np.insert(y, len(y), 0)

        plt.plot(x, y)

    def paintLines(self,plt):
        start = math.pi / 4
        end = math.pi * 3 / 4
        delta = math.pi / 4

        angle = start + delta
        plt.plot([0, 6 * math.cos(angle)], [0, 6 * math.sin(angle)], c='black')

        distances=[2,10/3,50/9]
        angle = np.arange(start, end, (end - start) / 100)
        for distance in distances:
            x = np.cos(angle) * distance
            y = np.sin(angle) * distance
            plt.plot(x, y, c='black')


    def update_point_cloud(self):
        #更新点云
        point_cloud = []
        if PointCloud.get_frame_pointcloud3(point_cloud, self.point_cloud_show_queue):
            print(point_cloud)
            origin_spots = [{'pos': np.array(point)} for point in point_cloud]
            self.origin_scatter.setData(origin_spots)

    #每一帧的显示
    def update_all(self):
        #跟踪器延迟
        if self.point_cloud_show_queue.qsize()==0:
            return

        self.update_point_cloud()

    # 启动定时器 时间间隔秒
    def timer_start(self):
        self.timer = pg.Qt.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_all)
        self.timer.start(10)

    def get_color_index(self):
        for i in range(20):
            if i not in self.used_color_indexes:
                self.used_color_indexes.append(i)
                return i


def visual_point_cloud(point_cloud_show_queue,xmin,xmax,ymax,detection_range):
    qapp = QtWidgets.QApplication(sys.argv)
    app = FourPlots(point_cloud_show_queue,xmin,xmax,ymax,detection_range)
    app.show()
    qapp.exec_()