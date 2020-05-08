from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

from point_cloud import PointCloud

class MainWindow():
    def __init__(self, x_range, y_range):
        self.app = QtGui.QApplication([])
        self.mw = QtGui.QMainWindow()
        self.mw.resize(x_range, y_range)
        self.view = pg.GraphicsLayoutWidget()  ## GraphicsView with GraphicsLayout inserted by default
        self.view.setBackground('w')
        self.mw.setCentralWidget(self.view)
        self.mw.show()
        self.mw.setWindowTitle('Show PointCloud')
        self.plot1 = self.view.addPlot() #xoy
        self.plot2 = self.view.addPlot() #yoz
        self.scatter1 = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.scatter2 = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.plot1.setRange(xRange=[-4, 4], yRange=[0, 7], padding=0)
        self.plot2.setRange(xRange=[0, 7], yRange=[0, 2.5], padding=0)

    def update_data(self):
        point_cloud = []
        frame_num, flag = PointCloud.get_frame_pointcloud(point_cloud, True)
        if flag:
            self.setWindowTitle('当前第：{0}帧'.format(frame_num))
            spots1 = [{'pos': np.array(point)} for point in point_cloud]
            spots2 = [{'pos': np.array([point[1], point[2]])} for point in point_cloud]
            self.scatter2.setData(spots2)
            self.scatter1.setData(spots1)
            #print(spots)
            self.plot1.addItem(self.scatter1)
            self.plot2.addItem(self.scatter2)

    def run(self):
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update_data)#定时调用plotData函数
        timer.start(1)#多少ms调用一次
        QtGui.QApplication.instance().exec_()