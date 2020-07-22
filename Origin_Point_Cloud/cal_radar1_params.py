# 作者     ：Origin_Point_Cloud
# 创建日期 ：2020-05-04  上午 11:49
# 文件名   ：receive_uart_data_录制数据.py


import struct
import math
import numpy as np
import os
import time
import sys
import serial
import json

from threading import Thread
from collections import OrderedDict
from queue import Queue
from copy import deepcopy

sys.path.append('../')

from Origin_Point_Cloud import common
from Origin_Point_Cloud.coordinate_transfer.compute_radar_pose import compute_radar1_direction
from Origin_Point_Cloud.utils import read_config,write_config

queue_for_calculate_polar = Queue()
queue_for_calculate_cart_transfer = Queue()

start_time = 0

tlv = "2I"
tlv_struct = struct.Struct(tlv)
tlv_size = tlv_struct.size

frame_header = "Q9I2H"
frame_header_struct = struct.Struct(frame_header)
frame_header_size = frame_header_struct.size

point_unit = "5f"
point_unit_struct = struct.Struct(point_unit)
point_unit_size = point_unit_struct.size

point = "2bh2H"
point_struct = struct.Struct(point)
point_size = point_struct.size

target_index = "B"
target_index_struct = struct.Struct(target_index)
target_index_size = target_index_struct.size

target_list = "I9f"
target_list_struct = struct.Struct(target_list)
target_list_size = target_list_struct.size

filepath=r'./data/radar_data'

class Point:
    """
    笛卡尔坐标系下的坐标点云
    """

    def __init__(self, pid, x, y, z, doppler, snr):
        self.pid = pid
        self.x = x
        self.y = y
        self.z = z
        self.doppler = doppler
        self.snr = snr


class RawPoint:
    """
    极坐标系下的坐标点云
    """

    def __init__(self, pid, azi, elev, range2, doppler, snr):
        self.pid = pid
        self.azi = azi
        self.elev = elev
        self.range2 = range2
        self.doppler = doppler
        self.snr = snr


class UartParseSDK():
    evm_queue = []
    evm_count = 0
    evm_current = 1
    save_path = ""
    end_size = 0
    current_size = 0
    _frame_num = 0
    save_to_queue_flag = True
    json_data_cart_transfer = OrderedDict()

    @classmethod
    def init_parameters(cls,end_size):
        cls.end_size = end_size
        print('正在接收数据...')
        cls.start_receive_data_thread().start()
        cls.save_data_thread().start()

    def __init__(self, user_flag, data_port, user_port, config_path, radar_z, theta):
        self.evm_num = UartParseSDK.evm_current
        UartParseSDK.evm_current += 1
        self.magic_word = 0x708050603040102
        self.bytes_data = bytes(1)
        self.max_points = 500
        self.polar = np.zeros((5, self.max_points))
        self.cart_transfer = np.zeros((5, self.max_points))
        self.detected_target_num = 0
        self.detected_point_num = 0
        self.target_list = np.ones((10, 20)) * (-1)
        self.target_index = np.zeros((1, self.max_points))
        self.fail = 0
        self.indexes = []
        self.frame_num = 0
        self.bytes_num = 4666
        self.tlv_header_length = 8
        self.header_length = 48
        self.missed_frame_num = 0
        self.theta = math.radians(theta)
        self.theta_diff = math.radians(90 - theta)
        self.theta_30 = math.radians(0)
        self.theta_15 = math.radians(0)
        self.radar_z = radar_z
        self.config_path = config_path
        self.queue_for_cart_transfer = Queue()
        self.user_flag = user_flag
        self.dict_cart_data=dict()


        '''
        port=串口号, 
        baudrate=波特率, 
        bytesize=数据位, 
        stopbits=停止位, 
        parity=校验位
        '''
        self.data_port = serial.Serial(data_port, 921600 * 1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                       timeout=0.025)
        self.user_port = serial.Serial(user_port, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                       timeout=0.3)
        if user_flag:
            UartParseSDK.evm_count += 1
            UartParseSDK.evm_queue.append(self)

    def receive_data_thread(self):
        """
        定义读取毫米波雷达采集数据的线程
        :return:接受数据线程
        """
        _receive_data_th = Thread(target=self.receive_data_th)
        return _receive_data_th

    def receive_data_th(self):
        """
        读取毫米波雷达采集数据
        :return: None
        """
        while not common.stop_flag:
            data = self.data_port.read(self.bytes_num)
            self.bytes_data += data
            self.bytes_data = self.get_frame(self.bytes_data)

    @staticmethod
    def save_data_thread():
        _save_data_thread = Thread(target=UartParseSDK.save_data_th)
        return _save_data_thread

    @classmethod
    def save_data_th(cls):
        while True:
            if cls.current_size >= cls.end_size:
                path_dir = filepath
                common.stop_flag = True
                time.sleep(1)
                for evm in range(len(cls.evm_queue)):
                    pdir=path_dir+'/'+str(evm)
                    if not os.path.isdir(pdir):
                        os.makedirs(pdir)
                    file = open(pdir + "/cart_data.json", "w")

                    json.dump(cls.evm_queue[evm].dict_cart_data, file)
                    file.flush()
                    file.close()
                print('正在计算雷达1在世界坐标系下的方向向量')
                try:
                    direction=compute_radar1_direction(filepath)
                    config=common.config
                    config.set('radar_params','radar1dx',str(direction[0]))
                    config.set('radar_params','radar1dy',str(direction[1]))
                    write_config(config)
                    print('雷达1在世界坐标系下的方向向量计算完毕')
                except:
                    raise Exception('请沿着雷达1右侧墙壁行走，最少30s')
                exit()

    def get_frame(self, data_in):
        """
        读取串口每一帧的数据
        :param data_in:串口数据
        :return:读取一帧后，剩下的数据
        """
        self.polar = np.zeros((5, self.max_points))
        self.cart_transfer = np.zeros((5, self.max_points))
        self.target = np.zeros((10, 20))
        self.detected_target_num = 0
        self.detected_point_num = 0
        while 1:
            try:
                magic, version, packet_length, plat_form, \
                frame_num, sub_frame_num, chirp_margin, frame_margin, \
                track_process_time, uart_sent_time, num_tlvs, checksum = \
                    frame_header_struct.unpack_from(data_in)
            except:
                self.fail = 1
                return data_in
            if magic != self.magic_word:
                data_in = data_in[1:]
            else:
                break
        if (self.frame_num + 1 != frame_num):
            self.missed_frame_num += 1
        self.frame_num = frame_num
        data_in = data_in[self.header_length:]
        left_data = packet_length - len(data_in) - self.header_length
        count = 0
        while left_data > 0 and count <= 3:
            new_data = self.data_port.read(left_data)
            left_data = packet_length - len(data_in) - self.header_length - len(new_data)
            data_in += new_data

        for i in range(num_tlvs):
            try:
                tlv_type, tlv_length = self.parse_tlv_header(data_in)
                data_in = data_in[self.tlv_header_length:]
                data_length = tlv_length - self.tlv_header_length
                if tlv_type == 6:
                    # point cloud
                    self.parse_point(data_in, data_length)
                elif tlv_type == 7:
                    # target list
                    self.parse_target_list(data_in, data_length)
                elif (tlv_type == 8):
                    # target index
                    self.parse_target_index(data_in, data_length)
                data_in = data_in[data_length:]
            except:
                pass
        return data_in

    def parse_tlv_header(self, data_in):
        """
        解析tlv
        :param data_in: 串口数据
        :return:tvl类型，该tlv类型对应的数据长度
        """
        tlv_type, tlv_length = tlv_struct.unpack_from(data_in)
        return tlv_type, tlv_length

    def parse_point(self, data_in, data_length):
        """
        解析point_unit
        :param data_in: 串口数据
        :param data_length: 串口数据长度
        :return: None
        """
        point_unit = point_unit_struct.unpack_from(data_in)
        data_in = data_in[point_unit_size:]
        # 剩余数据总长度
        self.detected_point_num = int((data_length - point_unit_size) / point_size)
        self.polar = np.zeros((5, self.detected_point_num))
        doppler_not_zero_num = 0
        for i in range(self.detected_point_num):
            try:
                elev, azi, doppler, ran, snr = point_struct.unpack_from(data_in)
                data_in = data_in[point_size:]
                if azi >= 128:
                    azi -= 256
                if elev >= 128:
                    elev -= 256
                if doppler >= 32768:
                    doppler -= 65536
                if doppler == 0:
                    continue
                doppler_not_zero_num += 1
                # range
                self.polar[0, i] = ran * point_unit[3]
                # azi
                self.polar[1, i] = azi * point_unit[1]
                # elev
                self.polar[2, i] = elev * point_unit[0]
                # doppler
                self.polar[3, i] = doppler * point_unit[2]
                # snr
                self.polar[4, i] = snr * point_unit[4]
            except:
                self.detected_point_num = i
                break
            finally:
                self.detected_point_num = doppler_not_zero_num
        self.polar_to_cart()

    def polar_to_cart(self):
        self.cart_transfer = np.empty((5, self.detected_point_num))
        cart_num = 0
        """
        # range
        self.polar[0, i]
        # azi
        self.polar[1, i]
        # elev
        self.polar[2, i]
        # doppler
        self.polar[3, i]
        # snr
        self.polar[4, i]
        """
        for i in range(0, self.detected_point_num):
            cart_num += 1
            # x
            self.cart_transfer[0, i] = self.polar[0, i] * math.cos(
                self.theta - self.polar[2, i] + self.theta_15) * math.sin(self.polar[1, i] - self.theta_30)
            # y
            self.cart_transfer[1, i] = self.polar[0, i] * math.cos(
                self.theta - self.polar[2, i] + self.theta_15) * math.cos(self.polar[1, i] - self.theta_30)
            # z
            self.cart_transfer[2, i] = self.radar_z - self.polar[0, i] * math.sin(
                self.theta - self.polar[2, i] + self.theta_15)
        self.cart_transfer[3, :] = self.polar[3, 0:self.detected_point_num]
        self.cart_transfer[4, :] = self.polar[4, 0:self.detected_point_num]
        temp = dict()
        temp["frame_num"] = self.frame_num
        temp["time_stamp"] = time.time() * 1000
        temp["point_num"] = cart_num
        temp["point_list"] = deepcopy(self.cart_transfer.transpose())
        self.queue_for_cart_transfer.put(deepcopy(temp))
        temp['point_list']=temp['point_list'].tolist()
        self.dict_cart_data[self.frame_num]=deepcopy(temp)
        UartParseSDK.current_size+=0.5

    def parse_target_list(self, data_in, data_length):
        """
        解析target
        :param data_in:串口数据
        :param data_length:串口数据长度
        :return:None
        """
        self.detected_target_num = int(data_length / target_list_size)
        target_list = np.empty((13, self.detected_target_num))
        for i in range(self.detected_target_num):
            target_data = target_list_struct.unpack_from(data_in)
            data_in = data_in[target_list_size:]
            # tid, posx, posy
            target_list[0:3, i] = target_data[0:3]
            # posz
            target_list[3, i] = target_data[7]
            # evlx, evly
            target_list[4:6, i] = target_data[3:5]
            # evlz
            target_list[6, i] = target_data[8]
            # accx, accy
            target_list[7:9, i] = target_data[5:7]
            # accz
            target_list[9, i] = target_data[9]
            target_list[10:13, i] = [0, 0, 0]
        self.target_list = target_list

    def parse_target_index(self, data_in, data_length):
        """
        解析target_index
        :param data_in: 串口数据
        :param data_length: 串口数据长度
        :return:None
        """
        self.detected_target_num = int(data_length / target_index_size)
        self.target_index = []
        for i in range(self.detected_target_num):
            index = target_index_struct.unpack_from(data_in)
            data_in = data_in[target_index_size:]
            self.target_index.append(index[0])

    @classmethod
    def start_receive_data_thread(cls):
        _start_receive_data_thread = Thread(target=cls.start_reveive_data_th)
        return _start_receive_data_thread

    @classmethod
    def start_reveive_data_th(cls):
        flag = True
        exit_flag = False
        while flag:
            for i in range(UartParseSDK.evm_count):
                attr = "receive_" + str(i + 1)
                setattr(UartParseSDK, attr, True)
                if not getattr(UartParseSDK, attr):
                    exit_flag = True
                    break
            if exit_flag:
                break
            else:
                for evm in cls.evm_queue:
                    evm.receive_data_thread().start()
                flag = False
                break

def spy_queue(queue1,queue2,queue3):
    while True:
        print(queue1.qsize(),queue2.qsize(),queue3.qsize())
        time.sleep(0.05)

if __name__ == "__main__":
    # UartParseSDK()函数参数说明：
    """
    arg_1:True代表启动该串口，False代表禁用该串口
    arg_2:数据串口
    arg_3:用户串口
    arg_4:配置文件目录
    arg_5:雷达板高度
    arg_6:雷达板倾角
    """

    # init_parameters()函数参数说明：
    """
    arg_1:0代表录制数据 -1实时运行
    arg_2:录制数据模式下，数据保存路径
    arg_3:录制数据的帧数
    arg_4:可视化模式（0：杨家辉点云可视化；1：杨家辉聚类可视化；2：郭泽中可视化）
    """
    time.sleep(5)

    frames=int(sys.argv[1])

    config=common.config

    for i in range(2):
        port=common.ports[i]
        configuration_file=common.configuration_files[i]
        height=float(config.get('radar_params','radar'+str(i+1)+'_height'))
        tilt=float(config.get('radar_params','radar'+str(i+1)+'_tilt'))
        UartParseSDK(True,port[0],port[1],configuration_file,height,tilt)
    time.sleep(1)
    UartParseSDK.init_parameters(frames)