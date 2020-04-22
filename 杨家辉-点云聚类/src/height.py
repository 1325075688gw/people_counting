import math

class Height():

	block_length=0.25
	# delta_height=[1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.2599404864574784, 1.1344320084145112,
	#               1.1988682786418468, 1.1119083755798245, 0.9852783189205492, 0.7072995609122058, 0.6282585815117643,
	#               0.3319765558510601, 0.08778045385836619, 0.008194747981847827, -0.25343017749817864,
	#               -0.37420679559517334, -0.4872027432969972, -0.6316345401074752, -0.701356690112346,
	#               -0.875, -1.05, -1.225, -1.4024673937450403, -1.5452798970619157]
	delta_height= [0, 0, 0.03, 0.1, 0.09, -0.01, 0.13, 0.14, 0.24, 0.34, 0.3, 0.35, 0.47, 0.41, 0.45, 0.49, 0.46, 0.45, 0.45, 0.51, 0.46, 0.54, 0.23, 0.25, 0.22, -0.11, -0.28]

	@staticmethod
	def get_people_height_list(cl):
		cluster_center_point_list = cl.get_cluster_center_point_list()
		cluster_height_list = cl.get_height_list()
		people_height_list = []
		for height, center_point in zip(cluster_height_list,cluster_center_point_list):
			x = center_point[0]
			y = center_point[1]
			distance = math.sqrt(x*x+y*y)
			people_height_list.append(Height.add_delta(height,distance))
		return people_height_list
			
	@staticmethod
	def add_delta(height,distance):
		if int(distance/Height.block_length)>=len(Height.delta_height):
			return height+Height.delta_height[-1]
		return height+Height.delta_height[int(distance/Height.block_length)]
