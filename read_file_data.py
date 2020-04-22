import json
from commo import frame_data_queue

def read_data():
	'''
	file=open(r'./data/data_4_18_2人一起前后，2-5米，5次/cart_transfer_data.json')
	data=json.load(file)
	print(data)
	for i in data:
		#print(temfile[i])
		frame_data_queue.put(data[i])
	'''
	filename = r'./data/格力/在雷达照射方向上来回走动/data_4_22_1_1.9_在雷达照射方向上来回走动/cart_transfer_data.json'
	with open(filename, 'r', encoding='utf-8') as f:
		temfile = json.load(f)
		#print(temfile)
		for i in temfile:
			#print(temfile[i])
			#print("test2")
			frame_data_queue.put(temfile[i])

