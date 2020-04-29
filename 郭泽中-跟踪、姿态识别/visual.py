from matplotlib import pyplot as plt
import numpy as np
import math
from PyQt5 import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import common

plt.rcParams['font.family'] = ['sans-serif']
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus']=False

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self,xmin,xmax,range):
        super().__init__()
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        layout = QtWidgets.QVBoxLayout(self._main)

        self.xmin=xmin
        self.xmax=xmax
        self.range=range

        #自定义部分
        dynamic_canvas = FigureCanvasQTAgg(plt.figure(figsize=(xmax*2,range)))
        layout.addWidget(dynamic_canvas)    #不要改

        self._dynamic_ax = dynamic_canvas.figure.subplots()
        #自定义部分end

        self._timer = dynamic_canvas.new_timer(
            0.000001, [(self._update_canvas, (), {})])
        self._timer.start()

    #每一帧的显示
    def _update_canvas(self):
        if common.loc_pos.empty():
            return

        locations,postures,clusters=common.loc_pos.get()

        self._dynamic_ax.clear()    #不要改

        self._dynamic_ax.set_xlim(self.xmin, self.xmax)
        self._dynamic_ax.set_ylim(0, self.range)
        angle = np.arange(math.pi / 6, math.pi * 5 / 6, math.pi / 60)
        x = np.cos(angle) * self.range
        y = np.sin(angle) * self.range
        x = np.insert(x, 0, 0)
        y = np.insert(y, 0, 0)
        x = np.insert(x, len(x), 0)
        y = np.insert(y, len(y), 0)
        self._dynamic_ax.plot(x, y)

        self._dynamic_ax.set_title('当前有'+str(len(postures))+'个人，聚类得到了'+str(len(clusters))+'个人')
        for person in locations:
            location=locations[person]
            posture=postures[person]
            self._dynamic_ax.plot(location[0],location[1],'o',markersize=40)
            self._dynamic_ax.text(location[0],location[1],str(posture))
        self._dynamic_ax.figure.canvas.draw()   #不要改

    def center(self):
        # 获得窗口
        qr = self.frameGeometry()
        # 获得屏幕中心点
        cp = QtWidgets.QDesktopWidget().availableGeometry().center()
        # 显示到屏幕中心
        qr.moveCenter(cp)
        self.move(qr.topLeft())
