import numpy as np  # 数据结构
import sklearn.cluster as skc
from sklearn.cluster import KMeans
import math
import time
from height import Height


class Cluster():
	def __init__(self,eps,minpts,type,min_cluster_count,cluster_snr_limit):
		self.eps = eps
		self.minpts = minpts
		self.type = type
		self.min_cluster_count = min_cluster_count
		self.cluster_division_limit = 80  #达到80个点，对点云分割（分割失败保持原样）
		self.frame_cluster_dict = {}
		self.cluster_snr_limit = cluster_snr_limit
		self.history_people_center_point_data = []
		self.history_data_length = 5

		#多帧融合
		self.mix_frame_num = 5
		self.frame_data_list = []  #[{"frame_num":1,"points":[]},..]
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
			point = [data['x'], data['y'], data['z'], data['doppler'], data['snr']]
			points.append(point)
		return points

	def cluster_filter_by_noise(self, tem_dict):
		#去掉噪声
		try:
			tem_dict.pop(-1)
		except:
			pass

	def cluster_filter_by_snr_sum(self, tem_dict):
		del_list = []
		for i in tem_dict:
			snr_sum = self.compute_cluster_sum_snr(tem_dict[i])
			if snr_sum < self.cluster_snr_limit:
				del_list.append(i)
		for key in del_list:
			tem_dict.pop(key)
	
	def cluster_filter_by_count(self, tem_dict, count):
		#去除点数较少的类
		del_list = []
		for i in tem_dict:
			if len(tem_dict[i]) < count:
				del_list.append(i)
		for key in del_list:
			tem_dict.pop(key)
	
	def compute_cluster_height(self, points):
		h = Height(1.5)
		height = h.compute_height(points)
		return height
	
	def compute_cluster_length_width_height(self, points):
		cluster_max = np.max(points, axis=0)
		cluster_min = np.min(points, axis=0)
		return cluster_max[0]-cluster_min[0],cluster_max[1]-cluster_min[1], cluster_max[1]-cluster_min[1]

	def compute_cluster_sum_snr(self, points):
		sum_snr = 0
		for point in points:
			sum_snr += point[4]
		return sum_snr

	def get_center_point(self, points):
		#取snr前10%的点计算点云中心
		k = int(len(points) * 0.1) + 1 #计算前10%点
		points.sort(key=lambda x: x[4], reverse=True)
		topsnr_points = [points[i] for i in range(k)]
		center_point = np.mean(points, axis=0)
		# center_point = []
		# range_points = []
		# for point in points:
		# 	if height-0.4 < point[2] < height:
		# 		range_points.append(point)
		# if len(range_points) > 0:
		# 	tem_center = np.mean(range_points, axis=0)
		# 	center_point = [tem_center[0], tem_center[1], origin_center[2]]
		# else:
		return [center_point[0],center_point[1],center_point[2]]

	def compute_cluster_attr(self, tem_dict):
		cluster_list = []
		k = 0
		for i in tem_dict:
			cluster_dict = {}
			center_point = np.mean(tem_dict[i], axis=0)
			varlist = np.var(tem_dict[i], axis=0)
			cluster_dict['cluster_id'] = k
			cluster_dict['points_num'] = len(tem_dict[i])
			#cluster_dict['center_point'] = [center_point[0], center_point[1], center_point[2]]
			cluster_dict['height'] = self.compute_cluster_height(tem_dict[i])
			cluster_dict['center_point'] = self.get_center_point(tem_dict[i])
			cluster_dict['dist'] = math.sqrt(cluster_dict['center_point'][0]**2 + cluster_dict['center_point'][1]**2)
			cluster_dict['var_x'] = varlist[0]
			cluster_dict['var_y'] = varlist[1]
			cluster_dict['var_z'] = varlist[2]
			cluster_dict['points'] = tem_dict[i]
			cluster_dict['length'], cluster_dict['width'], cluster_dict['body_height'] = self.compute_cluster_length_width_height(tem_dict[i])
			cluster_dict['sum_snr'] = self.compute_cluster_sum_snr(tem_dict[i])
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

	def cluster_filter_by_snr(self, coefficient, tem_dict):
		#取snr前百分之多少的点（coefficient），来代表这个人
		for i in tem_dict:
			#print("过滤前", len(tem_dict[i]))
			tem_dict[i].sort(key=lambda x: x[4], reverse=True)
			end_index = math.ceil(len(tem_dict[i]) * coefficient)
			tem_dict[i] = tem_dict[i][:end_index]
			#print("过滤后", len(tem_dict[i]))

	#开始聚类分割
	def compute_cluster_people_count(self, tem_dict, threshold):
		#求每个类对应的人数
		cluster_people_num = {}  #求每个类对应的上一帧的人的id号，对应多个表示有多个类聚成一个
		cluster_center_point_dict = {}  #求聚类中心点
		for i in tem_dict:
			cluster_center_point_dict[i] = self.get_center_point(tem_dict[i])
			cluster_people_num[i] = []
		dist_person_to_cluster_list = []  #上一帧的人 和这一帧的聚类的对应关系

		locations = {}
		for cluster in self.frame_cluster_dict['cluster']:
			locations[cluster['cluster_id']] = cluster['center_point']
		#求上一帧的人，在这帧当中，距离人最近的那个类
		for key in locations:
			dist_person_to_cluster = {} #{person_id:id,min_dist_id:id,min_dist:dist}
			min_dist = 1000
			for i in cluster_center_point_dict:
				dist_person_to_cluster['person_id'] = key
				cluster_dist = math.sqrt((cluster_center_point_dict[i][0]-locations[key][0])**2 +
											(cluster_center_point_dict[i][1]-locations[key][1])**2)
				if cluster_dist < min_dist:
					min_dist = cluster_dist
					dist_person_to_cluster['min_dist_id'] = i
					dist_person_to_cluster['min_dist'] = min_dist
			if len(dist_person_to_cluster) != 0:
				dist_person_to_cluster_list.append(dist_person_to_cluster)
		#print("距离上一帧的人的最小距离",dist_person_to_cluster_list)
		#如果距离人的距离小于阈值，那么就把这个人分配给对应的类
		#print(dist_person_to_cluster_list)
		for dist_person_to_cluster in dist_person_to_cluster_list:
			if dist_person_to_cluster['min_dist'] < threshold:
				cid = dist_person_to_cluster['min_dist_id']
				cluster_people_num[cid].append(dist_person_to_cluster['person_id'])
		return cluster_people_num

	def satisfy_divide(self, person_ids, cluster):
		#将聚类对应上一帧的人的点云合并，然后根据聚类点云和他对应的上一帧的人的总点云相似度，判断是否能分割
		people_point_cloud = []
		for pid in person_ids:
			people_point_cloud += self.frame_cluster_dict['cluster'][pid]['points']

		#聚类的点数应该大于其people_point_cloud的点数的80%,小于120%
		# if len(cluster) > 1.5*len(people_point_cloud) or len(cluster) < 0.5*len(people_point_cloud) :
		# 	print("不满足点数要求，无法分割")
		# 	return False

		#聚类的中心点，应该和其对应人的中心点的距离不超过0.3
		people_center_point = np.mean(people_point_cloud, axis=0)
		cluster_center_point = np.mean(cluster, axis=0)
		dist = math.sqrt((people_center_point[0]-cluster_center_point[0])**2 +
					(people_center_point[1]-cluster_center_point[1])**2)
		# if dist > 0.3:
		# 	print("不满足距离要求，无法分割")
		# 	return False

		#聚类横向距离不超过0.5，纵向距离不超过0.2
		if abs(people_center_point[0]-cluster_center_point[0]) > 0.5 or abs(people_center_point[1]-cluster_center_point[1]) > 0.2:
			return False

		#聚类的宽度和长度中，应该和对应人的长，宽类似
		plength, pwidth, pheight = self.compute_cluster_length_width_height(people_point_cloud)
		clength, cwidth, pheight = self.compute_cluster_length_width_height(cluster)
		#print("长：",abs(plength-clength),"宽：",abs(pwidth-cwidth))
		if abs(plength-clength) > 0.5 or abs(pwidth-cwidth) > 0.3:
			print("不满足长宽要求，无法分割")
			return False

		#如果聚类本身小于1米宽，不分
		if clength < 1:
			return False
		return True

	def divide_cluster_by_kmeans(self, k, points, origin_cluster_dict, max_id):
		#将kmeans得到的聚类，按照max_id+1进行编号，存入origin_cluster_dict当中
		X = np.array(points)
		tag = KMeans(k).fit_predict(X[:, :2])
		for j in range(len(tag)):
			tag[j] += max_id + 1
		divided_cluster = self.cluster_by_tag(points, tag)
		for j in divided_cluster:
			origin_cluster_dict[j] = divided_cluster[j]

	def divide_cluster_by_people_count(self, tem_dict):
		do_divide_flag = False
		if len(self.frame_cluster_dict) < 2 or len(self.frame_cluster_dict['cluster']) == 0:
			return
		#对这一帧匹配了多个上一帧的类的聚类，进行判断，看是否满足是上一帧多个类的融合，如果满足
		#将人数较多的类用kmeans分割
		cluster_people_num = self.compute_cluster_people_count(tem_dict, 1)
		#print(cluster_people_num)
		max_id = -1
		for i in tem_dict:
			if i > max_id:
				max_id = i
		del_id = []
		for i in cluster_people_num:
			#if len(cluster_people_num[i]) > 1:
			if len(cluster_people_num[i]) > 1:
				print("帧号", self.frame_cluster_dict['frame_num'], "准备分割")
			if len(cluster_people_num[i]) > 1 and self.satisfy_divide(cluster_people_num[i], tem_dict[i]):
				print("帧号", self.frame_cluster_dict['frame_num'], "分割成功")
				do_divide_flag = True
				k = len(cluster_people_num[i])
				del_id.append(i)
				self.divide_cluster_by_kmeans(k, tem_dict[i], tem_dict, max_id)
				max_id += k
		for key in del_id:
			tem_dict.pop(key)
		return do_divide_flag
	#聚类分割结束

	def people_check(self, tem_dict):
		#判断点云是否满足人的要求,不满足要求的删除
		del_list = []
		for i in tem_dict:
			height = self.compute_cluster_height(tem_dict[i])
			center_point = np.mean(tem_dict[i], axis=0)
			length, width, body_height = self.compute_cluster_length_width_height(tem_dict[i])
			if body_height < height-center_point[1]/3-0.2:  #根据雷达倾角得到的人的体长关系
				del_list.append(i)
				print("帧号：",self.frame_cluster_dict['frame_num'],
					  "height", height,
					  "y", center_point[2],
					  "最小体长", height-center_point[2]/3-0.2,"体长：",body_height,"删除成功")
		for i in del_list:
			tem_dict.pop(i)

	def get_topz_points(self, points):
		k = int(len(points) * 0.1) + 1 #计算前10%点
		points.sort(key=lambda x: x[2], reverse=True)
		topz_points = [points[i] for i in range(k)]
		X = np.array(topz_points)[:,:2]
		db = skc.DBSCAN(eps=self.eps, min_samples=5).fit(X)
		tags = db.labels_
		topz_cluster_dict = self.cluster_by_tag(points,tags)
		self.cluster_filter_by_noise(topz_cluster_dict)

		#如果有人头中心点距离较近，则不进行分割
		center_point_list = []
		for i in topz_cluster_dict:
			center_point = np.mean(topz_cluster_dict[i], axis=0)
			center_point_list.append(center_point)
		#计算相邻中心点的距离，有距离小于0.5的两个中心点，不进行分割
		dist_list = []
		if len(center_point_list)>=2:
			for i in range(1,len(center_point_list)):
				dist_list.append(math.sqrt((center_point_list[i-1][0]-center_point_list[i][0])**2+
										   (center_point_list[i-1][1]-center_point_list[i][1])**2))
		for dist in dist_list:
			if dist < 0.8:
				print("距离太近  不通过人头分")
				return -1
		return len(topz_cluster_dict)

	def divide_cluster_by_topz_points(self, tem_dict, do_divide_flag):
		if do_divide_flag:
			print("已经做过分割，不用进行再次分割")
			return
		max_id = -1
		for i in tem_dict:
			if i > max_id:
				max_id = i
		del_list = []
		origin_cluster_id = []
		for i in tem_dict:
			origin_cluster_id.append(i)
		for i in origin_cluster_id:
			k = self.get_topz_points(tem_dict[i])
			if k > 1:
				print("通过人头将聚类分割成%d个"%k)
				self.divide_cluster_by_kmeans(k, tem_dict[i], tem_dict, max_id)
				max_id += k
				del_list.append(i)
		for key in del_list:
			tem_dict.pop(key)

	def is_person(self, points):
		snr_sum = self.compute_cluster_sum_snr(points)
		#snr过小 不是人
		if snr_sum < self.cluster_snr_limit:
			return False
		#点数过少，不是人
		if len(points) < 20:
			return False
		return True

	def cluster_filter_by_latest_frame_data(self, tem_dict, min_repeat_time):
		#通过最近5帧信息，对这一帧的聚类进行过滤
		#search_dist = [0.2, 0.2, 0.25, 0.3, 0.35]
		search_dist = [0.35, 0.3, 0.25, 0.2, 0.2]
		tem_dict_count = {}  #统计这一帧的聚类出来的点在过去出现了多少次，过滤出现次数过少的类（认为是噪声）
		assigned = [[False for j in range(len(center))] for center in self.history_people_center_point_data] #判断这一帧的类是否被分配过，每个人只能被分配一次
		filter_id_list = []
		for i in tem_dict:
			tem_dict_count[i] = 0
			center_point = self.get_center_point(tem_dict[i])
			for j in range(len(self.history_people_center_point_data)):
				dist_list = []
				dist_index_list = []
				for k in range(len(self.history_people_center_point_data[j])):
					if assigned[j][k]:
						continue
					dist = math.sqrt((self.history_people_center_point_data[j][k][0]-center_point[0])**2 +
									 (self.history_people_center_point_data[j][k][1]-center_point[1])**2)
					dist_list.append(dist)
					dist_index_list.append(k)
				if len(dist_list) == 0:
					continue
				min_index = dist_index_list[0]
				min_dist = dist_list[0]
				for p in range(1, len(dist_list)):
					if min_dist > dist_list[p]:
						min_dist = dist_list[p]
						min_index = dist_index_list[p]
				if min_dist < search_dist[j]:
					tem_dict_count[i] += 1
					assigned[j][min_index] = True
		#将这一帧被分配次数小于2的类，并且这个类比较像噪声（不像人）
		for i in tem_dict:
			if tem_dict_count[i] < min_repeat_time and not self.is_person(tem_dict[i]):
				filter_id_list.append(i)

		for key in filter_id_list:
			tem_dict.pop(key)
			print("不像人，分配次数太少，删除这个类")


	def update_history_people_center_point_data(self):
		center_point_list = self.get_cluster_center_point_list()
		self.history_people_center_point_data.append(center_point_list)
		if len(self.history_people_center_point_data) <= self.history_data_length:
			return
		self.history_people_center_point_data.pop(0)


	def points_to_cluster_by_tag(self, points, tag):
		tem_dict = self.cluster_by_tag(points, tag)
		#print(tem_dict)		#将点云转换为人

		self.cluster_filter_by_noise(tem_dict)
		#按照点数和dopper对聚类进行分割
		#self.divide_cluster_by_count_dopper(tem_dict)
		#self.divide_cluster_by_count_dopper_v2(tem_dict)
		#过滤
		self.cluster_filter_by_count(tem_dict, self.min_cluster_count)
		#self.cluster_filter_by_count_and_distance(tem_dict)
		do_divide_flag = self.divide_cluster_by_people_count(tem_dict)
		#self.divide_cluster_by_topz_points(tem_dict, do_divide_flag)
		self.cluster_filter_by_snr(0.8, tem_dict)
		#self.cluster_filter_by_snr_sum(tem_dict)
		if len(self.history_people_center_point_data) == self.history_data_length:
			self.cluster_filter_by_latest_frame_data(tem_dict,1)
		#self.people_check(tem_dict)
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
		#print("begin",self.locations)
		self.frame_cluster_dict['frame_num'] = frame_data['frame_num']
		#print(frame_data['frame_num'])
		points = self.frame_data_to_cluster_points(frame_data)
		# frame_data_dict = {}
		# frame_data_dict['frame_num']
		# if len(self.frame_data_list) < self.mix_frame_num:
		# 	fram
		tag = self.dbscan_official(points)
		self.frame_cluster_dict['cluster'] = self.points_to_cluster_by_tag(points, tag)
		self.update_history_people_center_point_data()
		#print(self.frame_cluster_dict)

	#test
	def show_snr(self):
		print("帧号：%d" % self.frame_cluster_dict['frame_num'])
		for cluster in self.frame_cluster_dict['cluster']:
			data = np.array(cluster['points'])
			print("points_num:", cluster['points_num'])
			print("cluster_id:", cluster['cluster_id'])
			print("center_point:", cluster['center_point'])
			print("all_snr:", cluster['sum_snr'])
			#print("all_snr", data[:, 4])

	def get_sum_snr_list(self):
		sum_snr_list = []
		for cluster in self.frame_cluster_dict['cluster']:
			sum_snr = cluster['sum_snr']/cluster['points_num']
			sum_snr_list.append(sum_snr)
		return sum_snr_list
