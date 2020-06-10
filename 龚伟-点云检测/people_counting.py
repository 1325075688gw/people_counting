# 作者     ：gw
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

sys.path.append(r"../杨家辉-点云聚类")
sys.path.append(r"../郭泽中-跟踪、姿态识别")


from visual import run
import analyze_radar_data
import common
import show_test
from qt_show import MainWindow
from cluster_show import ClusterWindow


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
    def __init__(self, data_port, user_port, config_path, radar_z, theta):
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
        self.theta = math.radians(theta)
        self.theta_diff = math.radians(90-theta)
        self.theta_30 = math.radians(0)
        self.theta_15 = math.radians(0)
        self.radar_z = radar_z
        self.config_path = config_path
        self.save_2_queue_flag = True
        '''
        port=串口号, 
        baudrate=波特率, 
        bytesize=数据位, 
        stopbits=停止位, 
        parity=校验位
        '''
        self.user_port = serial.Serial(user_port, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                       timeout=0.3)
        self.data_port = serial.Serial(data_port, 921600 * 2, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                       timeout=0.025)

    def open_port(self):
        """
        打开串口
        :return: None
        """
        if self.data_port.isOpen():
            self.data_port.reset_output_buffer()
            print("数据串口打开成功！")
        else:
            print("数据串口打开失败！")

        if self.user_port.isOpen():
            self.user_port.reset_input_buffer()
            print("用户串口打开成功！")
        else:
            print("用户串口打开失败！")

    def send_config(self):
        """
        向毫米波雷达发送数据
        :return: None
        """
        current_dir = os.path.dirname(__file__)
        file = open(current_dir + self.config_path, "r+")
        if file is None:
            print("配置文件不存在!")
            return
        for text in file.readlines():
            if text == "\n":
                continue
            print("send config:" + text)
            self.user_port.write(text.encode('utf-8'))
            self.user_port.write('\n'.encode('utf-8'))
            time.sleep(0.2)
        file.close()

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

    def put_queue_th(self, save_flag, save_path, frame_nums, second):
        """
        毫米波雷达采集数据，然后将数据push到队列中，供杨家辉调用
        :return:None
        """
        start_time_frame = 0
        end_time_frame = 0
        while not common.stop_flag:
            if self.frame_num < 70:
                # print("frame_num:{0}".format(self.frame_num))
                continue
            point_cloud_num = 0
            point_cloud_list = []
            polar = queue_for_calculate_polar.get().transpose()

            for index, value in enumerate(polar):
                raw_point = RawPoint(index+1, value[0], value[1], value[2], value[3], value[4]).__dict__
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

            point_cloud_num = 0
            point_cloud_list = []
            cart_transfer = queue_for_calculate_cart_transfer.get().transpose()
            for index, value in enumerate(cart_transfer):
                point = Point(index + 1, value[0], value[1], value[2], value[3], value[4]).__dict__
                point_cloud_list.append(point)
                point_cloud_num += 1
            temp2 = dict()
            temp2["frame_num"] = self.frame_num
            temp2["time_stamp"] = t
            temp2["point_num"] = point_cloud_num
            temp2["point_list"] = point_cloud_list
            frame_num = "frame_num_" + str(self.frame_num)
            frame_dict_cart = {frame_num: temp2}
            print("frame_num:{0}".format(self.frame_num))
            if self.save_2_queue_flag == True:
                self.json_data_polar.update(frame_dict_polar)
                self.json_data_cart_transfer.update(frame_dict_cart)
            common.queue_for_cluster_transfer.put(temp2)

            if save_flag == -1:
                continue
            else:
                if save_flag == 0:
                    if self.frame_num == frame_nums:
                        self.save_2_queue_flag = False
                        polar_copy = deepcopy(self.json_data_polar)
                        cart_transfer_copy = deepcopy(self.json_data_cart_transfer)
                        self.json_data_polar.clear()
                        self.json_data_cart_transfer.clear()
                        self.save_data_thread(polar_copy, cart_transfer_copy, save_path).start()
                elif save_flag == 1:
                    end_time = time.time()/1000
                    if end_time-start_time >= second:
                        self.save_2_queue_flag = False
                        polar_copy = deepcopy(self.json_data_polar)
                        cart_transfer_copy = deepcopy(self.json_data_cart_transfer)
                        self.json_data_polar.clear()
                        self.json_data_cart_transfer.clear()
                        self.save_data_thread(polar_copy, cart_transfer_copy, save_path).start()
                else:
                    raise Exception("参数设置错误，请设置0,或者1")

    def put_queue_thread(self, save_flag, save_path, frame_nums, second):
        """
        定义将毫米波雷达采集到数据放到队列中，供杨家辉调用【线程】
        :return: None
        """
        _put_queue_th = Thread(target=self.put_queue_th, args=(save_flag, save_path, frame_nums, second))
        return _put_queue_th

    def save_data_th(self, polar_data, cart_data, save_path):
        path_dir = save_path
        common.stop_flag = True
        if not os.path.isdir(path_dir):
            print("创建文件夹：{0}".format(path_dir))
            os.makedirs(path_dir)
        file = open(path_dir + "/polar_data.json", "w")
        json.dump(polar_data, file)
        file.flush()
        file.close()
        print("极坐标数据写入完毕")

        file = open(path_dir + "/cart_transfer_data.json", "w")
        json.dump(cart_data, file)
        file.flush()
        file.close()
        print("转换坐标后的笛卡尔数据写入完毕")

        print("数据录制完成")

    def save_data_thread(self, polar_data, cart_data, save_path):
        _save_data_thread = Thread(target=self.save_data_th, args = (polar_data, cart_data, save_path))
        return _save_data_thread

    def show_frame(self):
        """
        先聚类，然后再可视化
        :return: None
        """
        time.sleep(1)
        # self.cluster_points_thread().start()
        # self.show_cluster_tracker_thread().start()

        # show_2d = Thread(target=show_test.run_show_pointcloud)
        # show_2d.start()

        # pass
        cluster_show = Thread(target=analyze_radar_data.cluster_points)
        cluster_show.start()
        #
        # cw = ClusterWindow(1200, 600)
        # cw.run()

        # mw = MainWindow(1200, 600)
        # mw.run()
        
        run(common.xmin, common.xmax, common.ymax)

    def cluster_points_thread(self):
        """
        定义聚类线程
        :return: 聚类线程
        """
        _cluster_points_th = Thread(target=analyze_radar_data.cluster_points)
        return _cluster_points_th

    def show_cluster_tracker_thread(self):
        """
        定义可视化线程
        :return:可视化线程
        """
        show_cluster_tracker_th = Thread(target=analyze_radar_data.show_cluster_tracker)
        return show_cluster_tracker_th

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
            except Exception as e:
                print("解析头出错：{0}".format(e))
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
        self.polar_to_cart()

    def polar_to_cart(self):
        self.cart_transfer = np.empty((5, self.detected_point_num))
        for i in range(0, self.detected_point_num):
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
        queue_for_calculate_polar.put(deepcopy(self.polar))
        queue_for_calculate_cart_transfer.put(deepcopy(self.cart_transfer))

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
    # 数据串口, 用户串口, 雷达高度, 雷达倾角
    uartParseSDK = UartParseSDK("COM4", "COM3", "./ODS_6m_default.cfg", 2.11, 11)
    uartParseSDK.open_port()
    uartParseSDK.send_config()
    uartParseSDK.receive_data_thread().start()
    uartParseSDK.put_queue_thread(0, r"./data/data_5_10,1-7米双人并排走，中间有坐下，第2次", 1200, 0 ).start()
    uartParseSDK.show_frame()
