import matplotlib.pyplot as plt
import numpy as np
import math

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle

plt.rcParams['font.family'] = ['sans-serif']
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus']=False

POSTURES_DICT = {1:"站立", 2:"坐着", 3:"躺着", 4:"行走"}


def show_track(locations, gestures, x, y):
	"""
	可视化程序
	:param locations: 位置
	:param gestures: 姿态
	:param x: 扇形绘图所需参数
	:param y: 扇形绘图所需参数
	:return: None
	"""
	plt.plot(x, y)
	plt.xlim(-4, 4)
	plt.ylim(0, 10)
	plt.title("当前检测到：{0}人".format(len(locations)))
	plt.plot(0,0,'o',label='雷达',markersize=40)
	for person in locations:
		location=locations[person]
		gesture=gestures[person]
		plt.plot(location[0],location[1],'o',markersize=40)
		plt.text(location[0],location[1],POSTURES_DICT[gesture])
	plt.legend(markerscale=1.0/10)
