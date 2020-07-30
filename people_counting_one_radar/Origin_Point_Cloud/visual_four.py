import numpy as np
import math,sys
from PyQt5.QtWidgets import QGridLayout,QWidget,QVBoxLayout,QMainWindow
import pyqtgraph as pg
from PyQt5 import QtWidgets

from Cluster.point_cloud import PointCloud

class FourPlots(QWidget):

    colors=['#FFB6C1','#228B22','#FFD700','#00FF7F','#FFFF00','#FFA500','#597ADF','#D72CFF',
            '#EDCE75','#5BB1AB','#844925','#3FBED1','#98352E','#243FB5','#3AF451','#8AF979',
            '#EB5EFA','#FDBB8D','#6388B6','#DB245D','#3C9643','#FCDC82','#481B33','#DAC9D4',
            '#F35595','#2B6D76']
    cluster_color = ['#FFB6C1', '#FFD700', '#00FF7F', '#FFFF00', '#FFA500', '#597ADF', '#D72CFF',
                     '#EDCE75', '#5BB1AB', '#844925', '#3FBED1', '#98352E', '#243FB5', '#3AF451', '#8AF979',
                     '#EB5EFA', '#FDBB8D', '#6388B6', '#DB245D', '#3C9643', '#FCDC82', '#481B33', '#DAC9D4',
                     '#F35595', '#2B6D76', '#228B22']
    used_color_indexes=[]

    posture_status={1:'站立',2:'坐着',3:'躺着',4:'行走'}

    def __init__(self,loc_pos,point_cloud_show_queue,cluster_show_queue,xmin,xmax,ymin,ymax,detection_range):
        super().__init__()

        self.loc_pos=loc_pos
        self.point_cloud_show_queue=point_cloud_show_queue
        self.cluster_show_queue=cluster_show_queue
        self.xmin=xmin
        self.ymin=ymin
        self.xmax=xmax
        self.ymax=ymax
        self.detection_range=detection_range

        self.people=dict()

        self.initUI()
        self.timer_start()

    def initUI(self):

        self.setGeometry(50,50,2*950*(self.xmax-self.xmin)/self.ymax,950)
        self.layout = QGridLayout(self)
        self.setLayout(self.layout)
        #初始化子图
        self.track_plt=pg.PlotWidget()
        self.center_plt=pg.PlotWidget()
        self.cluster_plt=pg.PlotWidget()
        self.point_plt=pg.PlotWidget()
        #设置子图坐标范围
        self.track_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[self.ymin,self.ymax],padding=0)
        self.center_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[self.ymin,self.ymax],padding=0)
        self.cluster_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[self.ymin,self.ymax],padding=0)
        self.point_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[self.ymin,self.ymax],padding=0)
        #将子图添加到主窗口
        self.layout.addWidget(self.track_plt,0,0)
        self.layout.addWidget(self.center_plt,0,1)
        self.layout.addWidget(self.cluster_plt,1,0)
        self.layout.addWidget(self.point_plt,1,1)
        #设置子图背景
        self.track_plt.setBackground('w')
        self.center_plt.setBackground('w')
        self.cluster_plt.setBackground('w')
        self.point_plt.setBackground('w')
        #初始化子图显示数据
        self.center_points=pg.ScatterPlotItem(size=40)
        self.track_points = pg.ScatterPlotItem(size=40)
        self.cluster_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.origin_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        #将子图显示数据与子图绑定
        self.track_plt.addItem(self.track_points)
        self.center_plt.addItem(self.center_points)
        self.cluster_plt.addItem(self.cluster_scatter)
        self.point_plt.addItem(self.origin_scatter)
        #初始化姿态列表
        self.texts=[]

    def paintBorder(self, plt):

        angle = np.arange(math.pi / 6, math.pi * 5 / 6, math.pi / 600)
        x = np.cos(angle) * self.detection_range
        y = np.sin(angle) * self.detection_range

        x = np.insert(x, 0, 0)
        y = np.insert(y, 0, 0)
        x = np.insert(x, len(x), 0)
        y = np.insert(y, len(y), 0)
        plt.plot(x, y)

    def update_track(self):
        # locations, postures, cluster_num, frame_num, origin_clusters = self.loc_pos.get()
        locations, heights, cluster_num, frame_num, origin_clusters = self.loc_pos.get()

        self.setWindowTitle('第' + str(frame_num) + '帧，当前帧有' + str(len(locations)) + '个人,当前帧有' + str(cluster_num) + '类')

        # 删除已消失掉的人
        to_be_deleted = []
        for person in self.people:
            if person not in locations:
                to_be_deleted.append(person)
        for person in to_be_deleted:
            for i in range(len(self.used_color_indexes)):
                if self.used_color_indexes[i] == self.people[person]:
                    del self.used_color_indexes[i]
                    break
            del self.people[person]

        for each in self.texts:
            self.track_plt.removeItem(each)
        self.texts = []

        locs = []
        for person in locations:

            if person not in self.people:
                index = self.get_color_index()
                self.people[person] = index

            locs.append({'pos': locations[person], 'brush': self.colors[self.people[person]]})
            text = pg.TextItem(self.posture_status[postures[person]], color='#000000')
            # text=pg.TextItem(html=('<h1>'+str(heights[person])+'</h1>'),color='#000000')
            text.setPos(locations[person][0], locations[person][1])

            self.texts.append(text)
            self.track_plt.addItem(text)

        origin_cluster = []
        for cluster in origin_clusters:
            origin_cluster.append({'pos': cluster})

        self.track_points.setData(locs)
        self.center_points.setData(origin_cluster)

    def update_cluster(self):
        cluster_result = self.cluster_show_queue.get()
        person_list = cluster_result['person_list']

        points_spots = []
        people_spots = []
        for i in range(len(person_list)):
            points_spots += [{'pos': np.array(point), 'brush': self.cluster_color[i]}
                             for point in person_list[i].points]
            people_spots += [
                {'pos': np.array([person_list[i].center_point[0], person_list[i].center_point[1]]), 'size': 50,
                 'brush': pg.mkBrush(255, 255, 255, 0),
                 'pen': {'color': self.cluster_color[i], 'width': 2}}]
        self.cluster_scatter.setData(points_spots)
        self.cluster_scatter.addPoints(people_spots)
        #更新点云
        point_cloud = []
        if PointCloud.get_frame_pointcloud3(point_cloud, self.point_cloud_show_queue):
            origin_spots = [{'pos': np.array(point)} for point in point_cloud]
            self.origin_scatter.setData(origin_spots)

    #每一帧的显示
    def update_all(self):
        if self.loc_pos.empty():
            return
        #跟踪器延迟
        if self.point_cloud_show_queue.qsize()==0 or self.cluster_show_queue.qsize()==0:
            return

        self.update_track()
        self.update_cluster()

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

def visual4plots(loc_pos,point_cloud_show_queue,cluster_show_queue,xmin,xmax,ymin,ymax,detection_range):
    qapp = QtWidgets.QApplication(sys.argv)
    app = FourPlots(loc_pos,point_cloud_show_queue,cluster_show_queue,xmin,xmax,ymin,ymax,detection_range)
    app.show()
    qapp.exec_()