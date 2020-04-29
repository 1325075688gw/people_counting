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
        self.plot = self.view.addPlot()
        self.scatter = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 255))
        self.plot.setRange(xRange=[-4, 4], yRange=[0, 7], padding=0)

    def update_data(self):
        point_cloud = []
        if PointCloud.get_frame_pointcloud(point_cloud, True):
            spots = [{'pos': np.array(point)} for point in point_cloud]
            self.scatter.setData(spots)
            #print(spots)
            self.plot.addItem(self.scatter)

    def run(self):
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update_data)#定时调用plotData函数
        timer.start(1)#多少ms调用一次
        QtGui.QApplication.instance().exec_()