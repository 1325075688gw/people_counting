class Points_Filter():
	def __init__(self, z_min, z_max, del_doppler, snr_limit):
		self.z_min = z_min
		self.z_max = z_max
		self.del_doppler = del_doppler
		self.snr_limit = snr_limit
	
	def filter_by_dopper(self, frame_data):
		point_list = []
		for point_dict in frame_data['point_list']:
			if point_dict['doppler'] != self.del_doppler:
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
		
	def snr_filter(self, frame_data):
		point_list = []
		for point_dict in frame_data['point_list']:
			if point_dict['snr'] > self.snr_limit:
				point_list.append(point_dict)
		frame_data['point_list'] = point_list
		frame_data['point_num'] = len(point_list)

	def run_filter(self, frame_data):
		self.filter_by_dopper(frame_data)
		self.filter_by_z(frame_data)
		self.snr_filter(frame_data)
		return frame_data['point_num']

