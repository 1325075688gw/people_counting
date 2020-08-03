# 作者     ：origin_point_cloud
# 创建日期 ：2020-05-04  上午 11:49
# 文件名   ：receive_uart_data_录制数据.py


import struct
import numpy as np
import os
import time
import sys
import serial

from collections import OrderedDict
from queue import Queue

sys.path.append('../')
from Origin_Point_Cloud import common

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

class UartParseSDK():
    evm_queue = []
    evm_count = 0
    evm_current = 0
    save_flag = -1
    save_path = ""
    end_size = 0
    current_size = 0
    _frame_num = 0
    save_to_queue_flag = True
    put_flag=1
    json_data_cart_transfer = [OrderedDict(),OrderedDict(),OrderedDict()]

    @classmethod
    def init_evm(cls):
        cls.open_port()
        cls.send_config()

    def __init__(self, user_flag, data_port, user_port, config_path):
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
        self.config_path = config_path
        self.user_flag = user_flag
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
            print('雷达'+str(evm.evm_num+1),end='')
            if evm.data_port.isOpen():
                evm.data_port.reset_output_buffer()
                print("数据串口打开成功！")
            else:
                print("数据串口打开失败！")
            print('雷达'+str(evm.evm_num+1),end='')
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
            print('雷达'+str(evm.evm_num+1)+'正在传输配置文件...')
            current_dir = os.path.dirname(__file__)
            file = open(current_dir + evm.config_path, "r+")
            if file is None:
                print("配置文件不存在!")
                return
            for text in file.readlines():
                if text == "\n":
                    continue
                evm.user_port.write(text.encode('utf-8'))
                evm.user_port.write('\n'.encode('utf-8'))
                time.sleep(0.2)
            file.close()
            print('雷达'+str(evm.evm_num+1)+'配置文件传输完成！')

def run_system():
    for i in common.evm_index:
        port=common.ports[i]
        configuration_file=common.configuration_files[i]
        UartParseSDK(True,port[0],port[1],configuration_file)

    UartParseSDK.init_evm()

if __name__ == "__main__":
    run_system()