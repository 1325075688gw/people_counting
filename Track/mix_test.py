import numpy as np
import math,sys
from PyQt5.QtWidgets import QGridLayout,QWidget
import pyqtgraph as pg
from PyQt5 import QtWidgets

class ApplicationWindow(QWidget):

    colors=['#FFB6C1','#228B22','#FFD700','#00FF7F','#FFFF00','#FFA500','#597ADF','#D72CFF',
            '#EDCE75','#5BB1AB','#844925','#3FBED1','#98352E','#243FB5','#3AF451','#8AF979',
            '#EB5EFA','#FDBB8D','#6388B6','#DB245D','#3C9643','#FCDC82','#481B33','#DAC9D4',
            '#F35595','#2B6D76']
    used_color_indexes=[]

    posture_status={1:'站立',2:'坐着',3:'躺着',4:'行走'}

    def __init__(self,xmin,xmax,ymax,detection_range,loc_pos):
        super().__init__()

        self.xmin=xmin
        self.xmax=xmax
        self.ymax=ymax
        self.detection_range=detection_range
        self.loc_pos=loc_pos

        self.people=dict()

        self.initUI()
        self.timer_start()

    def initUI(self):

        self.setGeometry(200,50,max(1500,950*(self.xmax-self.xmin)/self.ymax),950)
        self.layout = QGridLayout(self)
        self.setLayout(self.layout)

        self.no_mix_plt=pg.PlotWidget()
        self.mix_plt=pg.PlotWidget()

        self.no_mix_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[0,self.ymax],padding=0)
        self.mix_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[0,self.ymax],padding=0)

        self.layout.addWidget(self.no_mix_plt,0,0)
        self.layout.addWidget(self.mix_plt,0,1)

        self.no_mix_plt.setBackground('w')
        self.mix_plt.setBackground('w')

        # self.paintBorder(self.plt)
        # self.paintBorder(self.origin_plt)

        self.no_mix_locations=pg.ScatterPlotItem(size=40)
        self.mix_locations = pg.ScatterPlotItem(size=40)

        self.no_mix_plt.addItem(self.no_mix_locations)
        self.mix_plt.addItem(self.mix_locations)

    #每一帧的显示
    def _update_canvas(self):
        if self.loc_pos.empty():
            return
        no_mix_locations,mix_locations=self.loc_pos.get()

        locs=[]
        for radar in no_mix_locations:
            for location in no_mix_locations[radar]:
                locs.append({'pos':location,'brush':self.colors[int(radar)]})
        mix_locs=[]
        for location in mix_locations:
            mix_locs.append({'pos':location})

        self.no_mix_locations.setData(locs)
        self.mix_locations.setData(mix_locs)

    # 启动定时器 时间间隔秒
    def timer_start(self):
        self.timer = pg.Qt.QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_canvas)
        self.timer.start(50)

    def get_color_index(self):
        for i in range(20):
            if i not in self.used_color_indexes:
                self.used_color_indexes.append(i)
                return i


def run_visual(xmin,xmax,ymax,detectioin_range,loc_pos):
    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow(xmin, xmax, ymax,detectioin_range,loc_pos)
    app.show()
    qapp.exec_()