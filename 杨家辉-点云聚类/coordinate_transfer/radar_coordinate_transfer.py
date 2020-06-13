import math
import numpy as np


# 数学推导
def compute_direction_vector(x1, y1, x2, y2):
    # 同一个向量在坐标系1下表示为(x1,y1),在坐标系2下表示为(x2,y2),求坐标系2相对于坐标系1的方向向量
    v1 = np.mat([[x1], [y1]])
    # 变换矩阵
    C = np.mat([[y2, x2], [-1*x2, y2]])
    # 求方向向量
    dv = np.linalg.inv(C)*v1
    # 类型转换
    np_nv = np.array(dv)
    return [np_nv[0][0], np_nv[1][0]]
    # return np.array(dv)[0][0]


def compute_relative_position(x0, y0, dx, dy, x1, y1):
    # 求雷达2相对于雷达1的位置
    # (x0,y0)为雷达1中的点，(x1,y1)为同样的点在雷达2中的位置，(dx,dy)为雷达2相对于雷达1的方向向量

    # 归一化方向向量
    d = math.sqrt(dx**2+dy**2)
    norm_dy = [dx/d, dy/d]  # 雷达2 y轴向量
    norm_dx = [norm_dy[1], -1*norm_dy[0]] # 雷达2 x轴向量
    position = [x0-norm_dx[0]*x1-norm_dy[0]*y1, y0-norm_dx[1]*x1-norm_dy[1]*y1]
    return position


def radar_coordinate_transfer(a, b, theta, r):
    # 已知坐标系2的在坐标系1下的方向向量(a,b)，已知坐标系2下某个点A的极坐标(theta, r)，
    # 求A在坐标系1下的直角坐标

    # 求雷达的某点直角坐标
    x0 = r*math.sin(theta)
    y0 = r*math.cos(theta)

    # 求过渡矩阵C
    d = math.sqrt(a**2+b**2)
    C = np.mat([[b/d, a/d], [-1*a/d, b/d]])

    # 类型转换
    np_res = np.array(C * np.mat([[x0], [y0]]))
    return [np_res[0][0], np_res[1][0]]


def radar_coordinate_transfer2(a, b, x0, y0):
    # 求A在坐标系1下的直角坐标

    # 求过渡矩阵C
    d = math.sqrt(a**2+b**2)
    C = np.mat([[b/d, a/d], [-1*a/d, b/d]])

    # 类型转换
    np_res = np.array(C * np.mat([[x0], [y0]]))
    return [np_res[0][0], np_res[1][0]]


def radar2_direction_in_global(x1, y1, x2, y2):
    # 求雷达2，在世界坐标下的方向向量,(x1,y1)为雷达1在世界坐标下的方向向量，(x2,y2)为雷达2在雷达1下的方向向量
    # 求雷达2方向向量点在世界坐标下的坐标
    point_in_global = radar_coordinate_transfer2(x1, y1, x2, y2)
    return [point_in_global[0], point_in_global[1]]


def radar2_position_in_global(radar1_x, radar1_y, radar1_dx,radar1_dy, radar2_position_in_radar1):
    tem = radar_coordinate_transfer2(radar1_dx,radar1_dy,radar2_position_in_radar1[0],radar2_position_in_radar1[1])
    radar2_global_position = [radar1_x+tem[0],radar1_y+tem[1]]
    return radar2_global_position


if __name__ == "__main__":
    pass
