import numpy as np
import math,sys
from PyQt5.QtWidgets import QGridLayout,QWidget,QVBoxLayout,QMainWindow
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

        self.setGeometry(50,50,2*950*(self.xmax-self.xmin)/self.ymax,950)
        self.layout = QGridLayout(self)
        self.setLayout(self.layout)
        self.plt=pg.PlotWidget()
        self.origin_plt=pg.PlotWidget()
        self.plt.setRange(xRange=[self.xmin,self.xmax],yRange=[0,self.ymax],padding=0)
        self.origin_plt.setRange(xRange=[self.xmin,self.xmax],yRange=[0,self.ymax],padding=0)
        self.layout.addWidget(self.plt,0,0)
        self.layout.addWidget(self.origin_plt,0,1)
        self.plt.setBackground('w')
        self.origin_plt.setBackground('w')

        # self.paintBorder(self.plt)
        # self.paintBorder(self.origin_plt)

        self.origin_clusters=pg.ScatterPlotItem(size=40)
        self.locations = pg.ScatterPlotItem(size=30)
        self.assigned=pg.ScatterPlotItem(size=30)
        self.plt.addItem(self.locations)
        self.plt.addItem(self.assigned)
        self.origin_plt.addItem(self.origin_clusters)
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
        # plt.plot(x,self.ymax-y)

    #每一帧的显示
    def _update_canvas(self):
        if self.loc_pos.empty():
            return

        locations,postures,cluster_num,frame_num,origin_clusters=self.loc_pos.get()
        # locations,heights,cluster_num,assignment,frame_num,origin_clusters=self.loc_pos.get()

        self.setWindowTitle('第'+str(frame_num)+'帧，当前帧有'+str(len(locations))+'个人,当前帧有'+str(cluster_num)+'类')

        #删除已消失掉的人
        to_be_deleted=[]
        for person in self.people:
            if person not in locations:
                to_be_deleted.append(person)
        for person in to_be_deleted:
            for i in range(len(self.used_color_indexes)):
                if self.used_color_indexes[i]==self.people[person]:
                    del self.used_color_indexes[i]
                    break
            del self.people[person]

        for each in self.texts:
            self.plt.removeItem(each)
        self.texts=[]

        locs=[]
        assigned=[]
        for person in locations:

            if person not in self.people:
                index=self.get_color_index()
                self.people[person]=index

            locs.append({'pos':locations[person],'brush':self.colors[self.people[person]]})
            text=pg.TextItem(self.posture_status[postures[person]],color='#000000')
            # text=pg.TextItem(html=('<h1>'+str(self.posture_status[postures[person]])+'</h1>'),color='#000000')
            # text=pg.TextItem(html=('<h1>'+str(heights[person])+'</h1>'),color='#000000')
            # text = pg.TextItem(str(locations[person]), color='#000000')
            text.setPos(locations[person][0],locations[person][1])

            self.texts.append(text)
            self.plt.addItem(text)

        origin_cluster = []
        for cluster in origin_clusters:
            origin_cluster.append({'pos': cluster})

        self.locations.setData(locs)
        self.assigned.setData(assigned)
        self.origin_clusters.setData(origin_cluster)

    # 启动定时器 时间间隔秒
    def timer_start(self):
        self.timer = pg.Qt.QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_canvas)
        self.timer.start(10)

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