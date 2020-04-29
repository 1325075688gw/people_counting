import numpy as np
import math,time,random
from PyQt5.QtWidgets import QGridLayout,QWidget,QVBoxLayout,QMainWindow
import common
import pyqtgraph as pg

class ApplicationWindow(QWidget):

    colors=['#FFB6C1','#228B22','#FFD700','#00FF7F','#FFFF00','#FFA500','#597ADF','#D72CFF',
            '#EDCE75','#5BB1AB','#844925','#3FBED1','#98352E','#243FB5','#3AF451','#8AF979',
            '#EB5EFA','#FDBB8D','#6388B6','#DB245D','#3C9643','#FCDC82','#481B33','#DAC9D4',
            '#F35595','#2B6D76']
    used_color_indexes=[]

    def __init__(self,xmin,xmax,range):
        super().__init__()

        self.xmin=xmin
        self.xmax=xmax
        self.range=range

        self.people=dict()

        self.initUI()
        self.timer_start()

    def initUI(self):

        self.setGeometry(200,200,1000,620)
        self.layout = QGridLayout(self)
        self.setLayout(self.layout)
        self.plt=pg.PlotWidget()
        self.layout.addWidget(self.plt)
        self.plt.setBackground('w')

        self.paintBorder()

        self.locations = pg.ScatterPlotItem(size=50)
        self.plt.addItem(self.locations)
        self.texts=[]

    def paintBorder(self):
        angle = np.arange(math.pi / 6, math.pi * 5 / 6, math.pi / 60)
        x = np.cos(angle) * self.range
        y = np.sin(angle) * self.range
        x = np.insert(x, 0, 0)
        y = np.insert(y, 0, 0)
        x = np.insert(x, len(x), 0)
        y = np.insert(y, len(y), 0)
        self.plt.plot(x,y)

    #每一帧的显示
    def _update_canvas(self):
        if common.loc_pos.empty():
            return

        locations,postures,clusters=common.loc_pos.get()

        #删除已消失掉的人
        for person in self.people:
            if person not in locations:
                del self.people[person]

        for each in self.texts:
            self.plt.removeItem(each)
        self.texts=[]

        locs=[]
        for person in locations:

            if person not in self.people:
                index=self.get_color_index()
                self.people[person]=self.colors[index]

            locs.append({'pos':locations[person],'brush':self.people[person]})
            text=pg.TextItem(str(postures[person]),color='#000000')
            text.setPos(locations[person][0],locations[person][1])
            self.texts.append(text)
            self.plt.addItem(text)

        self.locations.setData(locs)

    # 启动定时器 时间间隔秒
    def timer_start(self):
        self.timer = pg.Qt.QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_canvas)
        self.timer.start(1)

    def get_color_index(self):
        for i in range(20):
            if i not in self.used_color_indexes:
                self.used_color_indexes.append(i)
                return i