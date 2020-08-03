# 作者     ：Origin_Point_Cloud
# 创建日期 ：2020-05-04  上午 11:49
# 文件名   ：receive_uart_data_录制数据.py


import struct
import numpy as np
import os
import time
import sys
import serial

from threading import Thread
from collections import OrderedDict
from queue import Queue
from copy import deepcopy

sys.path.append('../')
from Origin_Point_Cloud import common
from Origin_Point_Cloud.calculate_elev_delta_radar_height import cal_elev_delta_radar_height

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
    def __init__(self, data_port, user_port, config_path):
        self.json_data_cart_transfer = OrderedDict()
        self.json_data_polar = OrderedDict()
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
        self.config_path = config_path
        self.save_2_queue_flag = True
        self.current_size = 0
        '''
        port=串口号, 
        baudrate=波特率, 
        bytesize=数据位, 
        stopbits=停止位, 
        parity=校验位
        '''
        self.user_port = serial.Serial(user_port, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                       timeout=0.3)
        self.data_port = serial.Serial(data_port, 921600 * 1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                       timeout=0.025)

    def receive_data_th(self):
        """
        读取毫米波雷达采集数据
        :return: None
        """
        global start_time
        start_time = time.time()/1000
        while not common.stop_flag:
            data = self.data_port.read(self.bytes_num)
            self.bytes_data += data
            self.bytes_data = self.get_frame(self.bytes_data)

    def receive_data_thread(self):
        """
        定义读取毫米波雷达采集数据的线程
        :return:接受数据线程
        """
        _receive_data_th = Thread(target=self.receive_data_th)
        return _receive_data_th

    def put_queue_th(self, save_flag,radar_index):
        """
        毫米波雷达采集数据，然后将数据push到队列中，供杨家辉调用
        :return:None
        """
        while not common.stop_flag:
            if self.frame_num < 100:
                continue
            point_cloud_num = 0
            point_cloud_list = []
            polar = queue_for_calculate_polar.get().transpose()

            for index, value in enumerate(polar):
                #     def __init__(self, pid, azi, elev, range2, doppler, snr):
                raw_point = RawPoint(index+1, value[1], value[2], value[0], value[3], value[4]).__dict__
                # raw_point = RawPoint(index+1, value[1], value[2], value[0], value[3], value[4]).__dict__
                point_cloud_list.append(raw_point)
                point_cloud_num += 1
            temp = dict()

            t = time.time() * 1000
            t = int(round(t))
            temp["frame_num"] = self.frame_num
            temp["time_stamp"] = t
            temp["point_num"] = point_cloud_num
            temp["point_list"] = point_cloud_list
            frame_num = "frame_num_" + str(self.frame_num)
            frame_dict_polar = {frame_num: temp}
            if self.save_2_queue_flag == True:
                self.json_data_polar.update(frame_dict_polar)
            self.current_size += 1

            if save_flag == -1:
                continue
            else:
                if save_flag == 0:
                    if self.current_size == self.end_size:
                        print("正在计算雷达"+str(radar_index+1)+"高度和倾角...")
                        polar_data = deepcopy(self.json_data_polar)
                        self.save_2_queue_flag = False
                        common.stop_flag = True
                        elev_delta,radar_height=cal_elev_delta_radar_height(polar_data, self.person_height)
                        print('雷达'+str(radar_index+1)+'高度和倾角计算完成')
                        config=common.config
                        config.set('radar_params','radar'+str(radar_index+1)+'_height',str(radar_height))
                        config.set('radar_params','radar'+str(radar_index+1)+'_tilt',str(elev_delta))
                        write_config(config)
                        print('雷达'+str(radar_index+1)+'高度和倾角保存完毕')

    def put_queue_thread(self, save_flag, end_size,radar_index):
        """
        定义将毫米波雷达采集到数据放到队列中，供杨家辉调用【线程】
        :return: None
        """
        self.end_size = end_size
        _put_queue_th = Thread(target=self.put_queue_th, args=(save_flag,radar_index,))
        return _put_queue_th

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
        self.detected_point_num = int((data_length - point_unit_size) / point_size)
        self.polar = np.zeros((5, self.detected_point_num))

        for i in range(self.detected_point_num):
            try:
                elev, azi, doppler, ran, snr = point_struct.unpack_from(data_in)
                data_in = data_in[point_size:]
                # range
                self.polar[0, i] = ran * point_unit[3]
                if azi >= 128:
                    azi -= 256
                if elev >= 128:
                    elev -= 256
                if doppler >= 65536:
                    doppler -= 65536
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
        queue_for_calculate_polar.put(deepcopy(self.polar))

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
            data_in = data_in[target_list_size:]
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


if __name__ == "__main__":

    radar_index=int(sys.argv[1])-1
    person_height=float(sys.argv[2])
    frames=int(sys.argv[3])
    UartParseSDK.person_height=person_height
    print('请在雷达'+str(radar_index+1)+'前方1.5m-6m的范围内行走...')
    time.sleep(5)

    common.stop_flag=False

    port=common.ports[radar_index]
    configuration_file=common.configuration_files[radar_index]

    uartParseSDK = UartParseSDK(port[0], port[1], configuration_file)
    uartParseSDK.receive_data_thread().start()
    put_queue_thread=uartParseSDK.put_queue_thread(0,frames,radar_index)

    put_queue_thread.start()
    put_queue_thread.join()