class Points_Filter():
	def __init__(self, z_min, z_max, del_dopper):
		self.z_min = z_min
		self.z_max = z_max
		self.del_dopper = del_dopper
	
	def filter_by_dopper(self, frame_data):
		point_list = []
		for point_dict in frame_data['point_list']:
			if point_dict['doppler'] != self.del_dopper:
				point_list.append(point_dict)
		frame_data['point_list'] = point_list
		frame_data['point_num'] = len(point_list)
		
	def filter_by_z(self, frame_data):
		point_list = []
		for point_dict in frame_data['point_list']:
			if  self.z_max > point_dict['z'] > self.z_min:
				point_list.append(point_dict)
		frame_data['point_list'] = point_list
		frame_data['point_num'] = len(point_list)
		

	def run_filter(self, frame_data):
		self.filter_by_dopper(frame_data)
		self.filter_by_z(frame_data)
		return frame_data['point_num']

	
