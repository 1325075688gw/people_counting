class Points_Filter():
	def __init__(self, z_min, z_max, del_doppler):
		self.z_min = z_min
		self.z_max = z_max
		self.del_doppler = del_doppler
	
	def filter_by_doppler(self, frame_data):
		"""
		根据doppler过滤
		:param frame_data: 每一帧的点云数据
		:return: None
		"""
		point_list = []
		for point_dict in frame_data['point_list']:
			if point_dict['doppler'] != self.del_doppler:
				point_list.append(point_dict)
		frame_data['point_list'] = point_list
		frame_data['point_num'] = len(point_list)
		
	def filter_by_z(self, frame_data):
		"""
		根据z进行过滤
		:param frame_data: 每一帧的点云数据
		:return: None
		"""
		point_list = []
		for point_dict in frame_data['point_list']:
			if  self.z_max > point_dict['z'] > self.z_min:
				point_list.append(point_dict)
		frame_data['point_list'] = point_list
		frame_data['point_num'] = len(point_list)
		

	def run_filter(self, frame_data):
		"""
		执行过滤
		:param frame_data: 每一帧的点云数据
		:return: 这一帧点数
		"""
		self.filter_by_doppler(frame_data)
		self.filter_by_z(frame_data)
		return frame_data['point_num']

	
