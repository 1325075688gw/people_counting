import numpy as np  # 数据结构


class Height():
	def __init__(self, weight):
		'''
		:param weight:过滤离群点的阈值（标准差）权值
		'''
		self.weight = weight

	def filter_outliers(self, z_list):
		#过滤离群点
		avg = np.average(z_list)
		std = np.std(z_list)
		height_list = []
		for z in z_list:
			if abs(z - avg) <= self.weight * std:
				height_list.append(z)
		return height_list

	def compute_height(self, cluster_points):
		k = int(len(cluster_points) * 0.1) + 1 #计算前10%点
		sorted_points = cluster_points
		sorted_points.sort(key=lambda x: x[2], reverse=True)
		z_list = [sorted_points[i][2] for i in range(k)]
		height_list = self.filter_outliers(z_list)
		return round(float(np.mean(height_list)), 3)
