class Points_Filter():
    def __init__(self, z_min, z_max, del_doppler, snr_limit, range_x_min, range_x_max, range_y_min, range_y_max):
        self.z_min = z_min
        self.z_max = z_max
        self.del_doppler = del_doppler
        self.snr_limit = snr_limit
        self.range_x_min = range_x_min
        self.range_x_max = range_x_max
        self.range_y_min = range_y_min
        self.range_y_max = range_y_max

    def filter_by_dopper(self, frame_data):
        point_list = []
        for point_dict in frame_data['point_list']:
            if point_dict[3] != self.del_doppler:
                point_list.append(point_dict)
        frame_data['point_list'] = point_list
        frame_data['point_num'] = len(point_list)

    def filter_by_z(self, frame_data):
        point_list = []
        for point_dict in frame_data['point_list']:
            if self.z_max > point_dict[2] > self.z_min:
                point_list.append(point_dict)
        frame_data['point_list'] = point_list
        frame_data['point_num'] = len(point_list)

    def snr_filter(self, frame_data):
        point_list = []
        for point_dict in frame_data['point_list']:
            if point_dict[4] > self.snr_limit:
                point_list.append(point_dict)
        frame_data['point_list'] = point_list
        frame_data['point_num'] = len(point_list)

    def filter_by_range(self, frame_data):
        point_list = []
        for point_dict in frame_data['point_list']:
            if self.range_x_min < point_dict[0] < self.range_x_max and self.range_y_min < point_dict[
                1] < self.range_y_max:
                point_list.append(point_dict)
        frame_data['point_list'] = point_list
        frame_data['point_num'] = len(point_list)

    def run_filter(self, frame_data):
        self.filter_by_dopper(frame_data)
        self.filter_by_z(frame_data)
        self.snr_filter(frame_data)
        self.filter_by_range(frame_data)
        return frame_data['point_num']

