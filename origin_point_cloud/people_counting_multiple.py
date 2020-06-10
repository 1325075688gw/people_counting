# 作者     ：origin_point_cloud
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

sys.path.append(r"../cluster")
sys.path.append(r"../track")

from visual_all import run_visual
import analyze_radar_data
import common
from cluster_show import show_cluster
from point_cloud_show import point_cloud_show
import multiprocessing

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
    evm_queue = []
    evm_count = 0
    evm_current = 1
    save_flag = -1
    save_path = ""
    end_size = 0
    current_size = 0
    _frame_num = 0
    save_to_queue_flag = True
    json_data_cart_transfer = OrderedDict()

    @classmethod
    def init_parameters(cls, save_flag, save_path, end_size, show_flag):
        cls.save_flag = save_flag
        cls.end_size = end_size
        cls.save_path = save_path
        # cls.open_port()
        # cls.send_config()
        cls.start_receive_data_thread().start()
        cls.mix_all_queue_thread().start()
        if save_flag==0:
            cls.save_data_thread().start()
        cls.show_frame(show_flag)
        """
        if not os.path.isdir(save_path):
            print("创建文件夹：{0}".format(save_path))
            os.makedirs(save_path)
        file = open(save_path + "/cart_transfer_data.json", "w")
        file.close()
        cls.save_path = save_path + "/cart_transfer_data.json"
        """

    def __init__(self, user_flag, data_port, user_port, config_path, radar_z, theta,relative_pos,direction):
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
        self.relative_pos=relative_pos
        self.direction=direction
        if np.linalg.norm(direction)==0:
            raise Exception('方向向量怎么可以为零向量呢？你是不是傻兄弟？')
        self.theta_x=math.acos(direction[0]/np.linalg.norm(direction))
        self.theta_y=math.acos(direction[1]/np.linalg.norm(direction))
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


    @classmethod
    def open_port(cls):
        """
        打开串口
        :return: None
        """
        for evm in cls.evm_queue:
            if evm.data_port.isOpen():
                evm.data_port.reset_output_buffer()
                print("数据串口打开成功！")
            else:
                print("数据串口打开失败！")
            if evm.user_port.isOpen():
                evm.user_port.reset_input_buffer()
                print("用户串口打开成功！")
            else:
                print("用户串口打开失败！")

    @classmethod
    def send_config(cls):
        """
        向毫米波雷达发送数据
        :return: None
        """
        for evm in cls.evm_queue:
            current_dir = os.path.dirname(__file__)
            file = open(current_dir + evm.config_path, "r+")
            if file is None:
                print("配置文件不存在!")
                return
            for text in file.readlines():
                if text == "\n":
                    continue
                print("send config:" + text[:-1])
                evm.user_port.write(text.encode('utf-8'))
                evm.user_port.write('\n'.encode('utf-8'))
                time.sleep(0.2)
            file.close()
            for i in range(UartParseSDK.evm_count):
                attr = "receive_" + str(i + 1)
                setattr(UartParseSDK, attr, True)
                print("{0}板设置成功，属性：{1}".format(i + 1, getattr(UartParseSDK, attr)))

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
        print("开始接收:EVM_{0}板数据".format(self.evm_num))
        while not common.stop_flag:
            data = self.data_port.read(self.bytes_num)
            self.bytes_data += data
            self.bytes_data = self.get_frame(self.bytes_data)

    @classmethod
    def mix_all_queue_thread(cls):
        _mix_all_queue_th = Thread(target=cls.mix_all_queue_th)
        return _mix_all_queue_th

    @classmethod
    def mix_all_queue_th(cls):
        while not common.stop_flag:
            point_list=[]
            frame_data=dict()

            for evm in cls.evm_queue:
                while evm.queue_for_cart_transfer.qsize() > 3:
                    print("队列{0}出队".format(evm.evm_num))
                    evm.queue_for_cart_transfer.get()
                evm_point_list = evm.queue_for_cart_transfer.get()
                point_list+=evm_point_list.tolist()
            cls._frame_num += 1
            cls.current_size += 1
            frame_data["point_list"] = point_list
            frame_data["frame_num"] = cls._frame_num

            frame_num = "frame_num_" + str(cls._frame_num)
            frame_dict_cart = {frame_num: deepcopy(frame_data)}
            cls.json_data_cart_transfer.update(deepcopy(frame_dict_cart))
            common.queue_for_cluster_transfer.put(deepcopy(frame_data))

    @staticmethod
    def save_data_thread():
        _save_data_thread = Thread(target=UartParseSDK.save_data_th)
        return _save_data_thread

    @classmethod
    def save_data_th(cls):
        while True:
            if cls.current_size >= cls.end_size:
                path_dir = cls.save_path
                common.stop_flag = True
                if not os.path.isdir(path_dir):
                    print("创建文件夹：{0}".format(path_dir))
                    os.makedirs(path_dir)
                file = open(path_dir + "/cart_data.json", "w")

                # print("cart_data:{0}".format(cls.json_data_cart_transfer))
                time.sleep(1)
                json.dump(deepcopy(cls.json_data_cart_transfer), file)
                file.flush()
                file.close()
                print("笛卡尔数据写入完毕")
                print("数据录制完成")
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

    #雷达的相对位置发生变化后需要修改
    def polar_to_cart(self):
        '''
        :function:
            1.将极坐标转换为直角坐标
            2.根据雷达摆放的相对位置对直角坐标进行转换使均位于同一直角坐标系中
            3.根据雷达摆放位置对雷达各自的探测范围进行限定，过滤掉其他雷达区域中的点
        :params:
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
        :return: None
        '''
        self.cart_transfer = [[],[],[],[],[]]
        for i in range(self.detected_point_num):
            if self.direction[1]==1:
                y=self.relative_pos[1]+self.polar[0,i]* math.cos(
                self.theta - self.polar[2, i]) * math.cos(self.polar[1, i])

                # if y>common.ymax/2:
                #     continue

                x = self.relative_pos[0] + self.polar[0, i] * math.cos(
                    self.theta - self.polar[2, i]) * math.sin(self.polar[1, i])
            else:
                y = self.relative_pos[1] - self.polar[0, i] * math.cos(
                    self.theta - self.polar[2, i]) * math.cos(self.polar[1, i])

                # if y<common.ymax/2:
                #     continue

                x=self.relative_pos[0]-self.polar[0, i] * math.cos(
                self.theta - self.polar[2, i]) * math.sin(self.polar[1, i])

            z = self.radar_z - self.polar[0, i] * math.sin(
                self.theta - self.polar[2, i])

            self.cart_transfer[0].append(x)
            self.cart_transfer[1].append(y)
            self.cart_transfer[2].append(z)
            self.cart_transfer[3].append(self.polar[3, i])
            self.cart_transfer[4].append(self.polar[4, i])

        self.cart_transfer=np.array(self.cart_transfer)
        self.queue_for_cart_transfer.put(deepcopy(self.cart_transfer.transpose()))

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

    @classmethod
    def show_frame(cls, show_flag):
        """
        先聚类，然后再可视化
        :return: None
        """
        time.sleep(1)

        # 杨家辉过滤后，显示原始点云
        if show_flag == 0:
            multiprocessing.Process(target=point_cloud_show,args=(750,900,common.queue_for_cluster_transfer,)).start()
        else:
            multiprocessing.Process(target=analyze_radar_data.cluster_points, args=(show_flag,common.queue_for_cluster_transfer,common.cluster_show_queue,common.loc_pos,common.point_cloud_show_queue,)).start()
            # 杨家辉过滤后，显示聚类结果
            if show_flag == 1:
                multiprocessing.Process(target=show_cluster,args=(1200,900,common.point_cloud_show_queue,common.cluster_show_queue,)).start()
            # 郭泽中可视化
            else:
                multiprocessing.Process(target=run_visual,args=(common.xmin,common.xmax,common.ymax,common.detection_range,common.loc_pos,)).start()

def spy_queue(queue1,queue2,queue3):
    while True:
        print('直角坐标队列:',queue1.qsize(),'位置姿态队列:',queue2.qsize(),'聚类结果队列:',queue3.qsize())
        time.sleep(1)

def run_system():
    for i in common.evm_index:
        port=common.ports[i]
        relative_pos=common.relative_poses[i]
        direction=common.directions[i]
        tilt=common.tilts[i]
        height=common.heights[i]
        UartParseSDK(True,port[0],port[1],'./radar_parameters.cfg',height,tilt,relative_pos,direction)

    UartParseSDK.init_parameters(-1,r'./data/data_6_9,修改新配置单人,第2次',1200,2)

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

    # evm_num=int(sys.argv[1])
    # radar_height=float(sys.argv[2])
    # radar_tilt=float(sys.argv[3])
    # run_mode=int(sys.argv[4])
    # visual_module=int(sys.argv[5])
    #
    # if len(sys.argv)>6:
    #     file_name=sys.argv[6]
    #     frame_num=int(sys.argv[7])
    # else:
    #     file_name=''
    #     frame_num=0

    # for i in range(evm_num):
    #     UartParseSDK(True,'/dev/ttyACM'+str(i*2),'/dev/ttyACM'+str(i*2+1),'./radar_parameters.cfg',radar_height,radar_tilt)

    run_system()

    # uartParseSDK1 = UartParseSDK(True, "COM3", "COM4", "./radar_parameters.cfg", 2.41, 10)
    # uartParseSDK2 = UartParseSDK(True, "COM13", "COM14", "./radar_parameters.cfg", 2.41, 10)
    # # UartParseSDK.init_parameters(run_mode, r"./data/"+file_name, frame_num, visual_module)
    # UartParseSDK.init_parameters(0,r'./data/data_6_6,瞎录,第2次',50,1)

    t=multiprocessing.Process(target=spy_queue,args=(common.queue_for_cluster_transfer,common.loc_pos,common.cluster_show_queue,))
    t.start()