import sys
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

sys.path.append(r"../杨家辉-点云聚类")
sys.path.append(r"../郭泽中-跟踪、姿态识别")


from common import cluster_show_queue
from common import point_cloud_show_queue

from point_cloud import PointCloud

class ClusterWindow():

    def __init__(self, x_range, y_range):
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        self.app = QtGui.QApplication([])
        self.mw = QtGui.QMainWindow()
        self.mw.resize(x_range, y_range)
        self.view = pg.GraphicsLayoutWidget()  ## GraphicsView with GraphicsLayout inserted by default
        self.view.setBackground('w')
        self.mw.setCentralWidget(self.view)
        self.mw.show()
        self.mw.setWindowTitle('Show Cluster')
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        print("oooooooooooofsdfvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")

        #绘制聚类效果图
        self.cluster_plot = self.view.addPlot() #cluster_show
        self.cluster_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.cluster_plot.setRange(xRange=[-4, 4], yRange=[0, 8], padding=0)
        self.cluster_color = ['#FFB6C1', '#FFD700', '#00FF7F', '#FFFF00', '#FFA500', '#597ADF', '#D72CFF',
            '#EDCE75', '#5BB1AB', '#844925', '#3FBED1', '#98352E', '#243FB5', '#3AF451', '#8AF979',
            '#EB5EFA', '#FDBB8D', '#6388B6', '#DB245D', '#3C9643', '#FCDC82', '#481B33', '#DAC9D4',
            '#F35595', '#2B6D76', '#228B22']
        self.frame_num_text = pg.TextItem()
        self.cluster_plot.addItem(self.frame_num_text)
        self.frame_num_text.setPos(-0.2, 8)
        self.cluster_plot.addItem(self.cluster_scatter)
        self.snr_texts_item = []

        #绘制原始点云
        self.origin_plot = self.view.addPlot() #xoy
        self.origin_scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.origin_plot.setRange(xRange=[-4, 4], yRange=[0, 8], padding=0)

    def update_data(self):

        #更新聚类数据
        print("vvvvvvvvvvv")
        point_cloud = []
        for text_item in self.snr_texts_item:
            text_item.setText("")
        print("fssssssssssqqqqqqqqqqqqqsss")
        if point_cloud_show_queue.empty() or cluster_show_queue.empty():
            return
        print("fsssssssssssss")
        if not cluster_show_queue.empty():
            cluster_data = cluster_show_queue.get()
            print("dddff:{0}".format(cluster_data))
            points_spots = []
            people_spots = []
            for cluster in cluster_data['cluster']:
                if cluster['cluster_id'] < len(self.snr_texts_item):
                    self.snr_texts_item[cluster['cluster_id']].setPos(cluster['center_point'][0], cluster['center_point'][1])
                    self.snr_texts_item[cluster['cluster_id']].setText("snr:%d" % cluster['sum_snr'], color='#000000')
                else:
                    snr_text = pg.TextItem()
                    self.snr_texts_item.append(snr_text)
                    self.cluster_plot.addItem(snr_text)

                points_spots += [{'pos': np.array(point), 'brush': self.cluster_color[cluster['cluster_id']]}
                                 for point in cluster['points']]
                people_spots += [{'pos': np.array(cluster['center_point']), 'size': 50,
                                  'brush': pg.mkBrush(255, 255, 255, 0),
                                  'pen': {'color': self.cluster_color[cluster['cluster_id']], 'width': 2}}]
            self.cluster_scatter.setData(points_spots)
            self.cluster_scatter.addPoints(people_spots)
            #print(("第%d帧" % cluster_data['frame_num']))
            self.frame_num_text.setText(("第%d帧" % cluster_data['frame_num']), color='#000000')

        #更新原始点云数据
        point_cloud = []
        if PointCloud.get_frame_pointcloud(point_cloud, True):
            origin_spots = [{'pos': np.array(point)} for point in point_cloud]
            self.origin_scatter.setData(origin_spots)
            self.origin_plot.addItem(self.origin_scatter)

    def run(self):
        timer = pg.QtCore.QTimer()
        print("vgcvvvvv")
        timer.timeout.connect(self.update_data)  # 定时调用plotData函数
        timer.start(1)  # 多少ms调用一次
        QtGui.QApplication.instance().exec_()