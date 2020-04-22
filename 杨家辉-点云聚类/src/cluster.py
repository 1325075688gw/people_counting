import numpy as np  # 数据结构
import sklearn.cluster as skc
import math

class Cluster():
	def __init__(self,eps,minpts,type,min_cluster_count):
		self.eps = eps
		self.minpts = minpts
		self.type = type
		self.min_cluster_count = min_cluster_count
		self.cluster_division_limit = 80  #达到80个点，对点云分割（分割失败保持原样）
		self.frame_cluster_dict = {}
		'''
		frame_cluster_dict
		{
			'frame_num':
			'cluster':[
						{'cluster_id'
						...
						},
						...
					  ]
		}
		'''
	def dbscan_official(self, data):
		if data == []:
			return []
		X = np.array(data)
		if self.type == '2D':
			X = X[:,:2]
		elif self.type == '3D':
			X = X[:,:3]
		db = skc.DBSCAN(eps=self.eps, min_samples=self.minpts).fit(X)
		return db.labels_
	
	def frame_data_to_cluster_points(self,frame_data):
		points = []
		for data in frame_data['point_list']:
			point = [data['x'],data['y'],data['z'],data['doppler'],data['snr']]
			points.append(point)
		return points
	
	def cluster_filter_by_noise(self, tem_dict):
		#去掉噪声
		try:
			tem_dict.pop(-1)
		except:
			pass
	
	def cluster_filter_by_count(self, tem_dict, count):
		#去除点数较少的类
		del_list = []
		for i in tem_dict:
			if len(tem_dict[i]) < count:
				del_list.append(i)
		for key in del_list:
			tem_dict.pop(key)
	
	def compute_cluster_height(self, points):
		k = int(len(points) * 0.1) + 1 #计算前10%点
		z_list = [points[i][2] for i in range(k)]
		return round(np.mean(z_list),3)
	
	def compute_cluster_length_width(self, points):
		cluster_max = np.max(points, axis=0)
		cluster_min = np.min(points, axis=0)
		return cluster_max[0]-cluster_min[0],cluster_max[1]-cluster_min[1]
	
	def compute_cluster_attr(self, tem_dict):
		cluster_list = []
		k = 0
		for i in tem_dict:
			cluster_dict = {}
			center_point = np.mean(tem_dict[i], axis=0)
			varlist = np.var(tem_dict[i], axis=0)
			cluster_dict['cluster_id'] = k
			cluster_dict['points_num'] = len(tem_dict[i])
			cluster_dict['center_point'] = [center_point[0],center_point[1],center_point[2]]
			cluster_dict['var_x'] = varlist[0]
			cluster_dict['var_y'] = varlist[1]
			cluster_dict['var_z'] = varlist[2]
			cluster_dict['height'] = self.compute_cluster_height(tem_dict[i])
			cluster_dict['points'] = tem_dict[i]
			cluster_dict['length'], cluster_dict['width'] = self.compute_cluster_length_width(tem_dict[i])
			cluster_list.append(cluster_dict)
			k += 1
		return cluster_list
	
	
	def cluster_by_tag(self, points, tag):
		tem_dict = {}
		#按照类别将点分类
		key_list = []
		for i in tag:
			if i not in key_list:
				key_list.append(i)
		#key_list = sorted(key_list)
		for i in key_list:
			tem_dict[i] = []
		for t, point in zip(tag, points):
			tem_dict[t].append(point)
		return tem_dict	
		
		
		
	def divide_cluster_by_count_dopper(self, tem_dict):
		'''
		对点数大于100的类，根据dopper单独用dbscan，保留大于40个点的类
		'''
		i = 0
		n = len(tem_dict)
		print(n)
		new_key = n
		while i < n:
			if len(tem_dict[i]) > 100:
				print(len(tem_dict[i]))
				X = np.array(tem_dict[i])
				#dbscan至少2维，这里给他主动将snr那一列设为同样的值，不影响聚类结果
				for x_array in X:
					x_array[4] = 0
				X = X[:,3:5]
				db = skc.DBSCAN(eps=0.15, min_samples=15).fit(X)
				divided_dict = self.cluster_by_tag(tem_dict[i], db.labels_)
				self.cluster_filter_by_noise(divided_dict)
				length_divided = len(divided_dict)
				self.cluster_filter_by_count(divided_dict, 40)
				print("go here")
				for j in divided_dict:
					tem_dict[j+new_key] = divided_dict[j]
				new_key += length_divided
				#删除分割的类
				tem_dict.pop(i)
			i += 1
			print("i,n:",i,n)
	
	def divide_cluster_by_count_dopper_v2(self, tem_dict):
		'''
		对点数大于2倍的类最小点数的类进行分割，按照doppler分割，分割后根据距离过滤
		'''
		i = 0
		n = len(tem_dict)
		print(n)
		new_key = n
		while i < n:
			center_point = np.mean(tem_dict[i], axis=0)
			dist = math.sqrt(center_point[0]*center_point[0] + center_point[1]*center_point[1])
			divide_points_limit = 10
			if len(tem_dict[i]) > divide_points_limit and (dist < 7 and len(tem_dict[i]) > 2*(1-dist/7)*self.min_cluster_count or dist >= 7):
				#print(len(tem_dict[i]))
				X = np.array(tem_dict[i])
				#dbscan至少2维，这里给他主动将snr那一列设为同样的值，不影响聚类结果
				for x_array in X:
					x_array[4] = 0
				X = X[:,3:5]
				db = skc.DBSCAN(eps=0.05, min_samples=5).fit(X)
				divided_dict = self.cluster_by_tag(tem_dict[i], db.labels_)
				self.cluster_filter_by_noise(divided_dict)
				length_divided = len(divided_dict)
				#self.cluster_filter_by_count(divided_dict, 40)
				#print("go here")
				for j in divided_dict:
					tem_dict[j+new_key] = divided_dict[j]
				new_key += length_divided
				#删除分割的类
				tem_dict.pop(i)
			i += 1
			#print("i,n:",i,n)
			
	def cluster_filter_by_count_and_distance(self, tem_dict):
		del_list = []
		for i in tem_dict:
			center_point = np.mean(tem_dict[i], axis=0)
			dist = math.sqrt(center_point[0]*center_point[0] + center_point[1]*center_point[1])
			if dist < 7 and len(tem_dict[i]) < (1-dist/7)*self.min_cluster_count:
				del_list.append(i)
		for key in del_list:
			tem_dict.pop(key)		
	
	def points_to_cluster_by_tag(self, points, tag):
		tem_dict = self.cluster_by_tag(points, tag)
		#print(tem_dict)
		self.cluster_filter_by_noise(tem_dict)
		#按照点数和dopper对聚类进行分割
		#self.divide_cluster_by_count_dopper(tem_dict)
		#self.divide_cluster_by_count_dopper_v2(tem_dict)
		#过滤
		#self.cluster_filter_by_count(tem_dict, self.min_cluster_count)
		self.cluster_filter_by_count_and_distance(tem_dict)
		#tem_dict = sorted(tem_dict.items(), key = lambda x : x[0]) #按key排序
		cluster_list = self.compute_cluster_attr(tem_dict)
		return cluster_list
	
	def get_cluster_center_point_list(self):
		cluster_center_point_list = []
		for cluster in self.frame_cluster_dict['cluster']:
			center_point = [cluster['center_point'][0],cluster['center_point'][1]]
			cluster_center_point_list.append(center_point)
		return cluster_center_point_list
	
	def get_height_list(self):
		cluster_height_list = []
		for cluster in self.frame_cluster_dict['cluster']:
			height = cluster['height']
			cluster_height_list.append(height)
		return cluster_height_list
	
	def show_cluster_dopper(self):
		#把聚好类的点云的dopper信息输出在终端，观察同一类中dopper的变化
		#如果同一类中有两个人dopper不同，就可以聚成2个类,目前正在测试
		print("帧号：%d"%self.frame_cluster_dict['frame_num'])
		for cluster in self.frame_cluster_dict['cluster']:
			dopper_list = []
			for point in cluster['points']:
				dopper_list.append(round(point[3],3))
			print(cluster['cluster_id'],dopper_list)
	
	
	
	def do_clsuter(self, frame_data):
		self.frame_cluster_dict['frame_num'] = frame_data['frame_num']
		points = self.frame_data_to_cluster_points(frame_data)
		tag = self.dbscan_official(points)
		self.frame_cluster_dict['cluster'] = self.points_to_cluster_by_tag(points, tag)
		#print(self.frame_cluster_dict)
		
	
	

