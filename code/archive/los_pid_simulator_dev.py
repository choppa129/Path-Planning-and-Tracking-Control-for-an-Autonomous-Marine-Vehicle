import socket
import json
import threading
import time
import numpy as np
import pandas as pd
import math

Host = '127.0.0.1'  # IP地址

Port_GPS = 60100  # GPS端口
Port_IMU = 60101  # IMU端口
Port_Buoy = 60103  # 浮标端口

Port_Control = 60199  # 控制端口
Port_Path = 60198  # 路径输出端口

socket_GPS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建UDP套接字
socket_IMU = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_Buoy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

socket_Control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_Path = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

socket_GPS.bind((Host, Port_GPS))  # 端口绑定
socket_IMU.bind((Host, Port_IMU))
socket_Buoy.bind((Host, Port_Buoy))

longitude = None  # 经度
latitude = None  # 纬度

pitch = None  # 俯仰角
roll = None  # 翻滚角
yaw = None  # 偏航角
velocity = None  # 速度

buoy_list = None  # 浮标位置

pre_error = 0
sum_error = 0
dt = 0.1


# GPS信息接收函数
def GPS_receive():
    global longitude, latitude
    while True:
        data, _ = socket_GPS.recvfrom(1024)
        data_json = json.loads(data.decode('utf-8'))
        longitude = data_json["longitude"]
        latitude = data_json["latitude"]


# IMU信息接收函数
def IMU_receive():
    global pitch, roll, yaw, velocity
    while True:
        data, _ = socket_IMU.recvfrom(1024)
        data_json = json.loads(data.decode('utf-8'))
        pitch = data_json["pitch"]
        roll = data_json["roll"]
        yaw = data_json["yaw"]
        velocity = data_json["velocity"]


# 浮标位置接收函数
def Buoy_receive():
    global start, goal, obstacle_list
    while True:
        data, _ = socket_Buoy.recvfrom(10240)
        data_json = json.loads(data.decode('utf-8'))

        start = list(data_json['start'].values())
        goal = list(data_json['end'].values())
        obstacle_list = []
        for i in range(len(data_json['obstacles'])):
            obstacle_list.append(list(data_json['obstacles'][i].values()))


# 发送路径
def Path_send(path_points):
    path = {}
    path["points"] = path_points
    path = json.dumps(path)
    path = path.encode('utf-8')
    socket_Path.sendto(path, (Host, Port_Path))


# 发送控制信号，throttle为油门，steer为方向，范围均为 -1 ~ 1
def Control_send(throttle, steer):
    control_cmd = {}
    control_cmd["throttle"] = throttle
    control_cmd["steer"] = steer
    control_cmd = json.dumps(control_cmd)
    control_cmd = control_cmd.encode('utf-8')
    socket_Control.sendto(control_cmd, (Host, Port_Control))


# PID控制艏向
def PID_controller(current_yaw, target_yaw):
    global pre_error, sum_error
    if (abs(target_yaw+360-current_yaw) > abs(target_yaw-current_yaw)):
        error = target_yaw - current_yaw
    else:
        error = target_yaw +360 - current_yaw
    #print(error)
    sum_error += error * dt
    d_error = (error - pre_error) / dt

    # PID控制参数
    #Kp = 0.01 # 速度0.2
    Kp = 0.03 # 速度0.8
    Ki = 0
    Kd = 0.01

    steer = Kp * error + Ki * sum_error + Kd * d_error
    steer = max(min(steer,1),-1)

    pre_error = error

    return steer


# LOS函数
def LOS(current_position, target_point, path_points, ratio=0.6):
    global path_index

    distance_to_target = calculate_distance(current_position['latitude'], current_position['longitude'],
                                            target_point['latitude'], target_point['longitude'])
    if distance_to_target < 0.005:
        path_index += 1
        if path_index >= len(path_points):
            path_index = len(path_points) - 1
        target_point = path_points[path_index]

    target_point_next = path_points[path_index - 1]

    if target_point_next['latitude'] != target_point['latitude']:
        # 计算两点间的斜率
        k = (target_point_next['longitude'] - target_point['longitude']) / (
                    target_point_next['latitude'] - target_point['latitude'])

        # 计算两点间的截距
        b = target_point_next['longitude'] - k * target_point_next['latitude']

        # 根据斜率和截距计算垂直点的坐标
        x_perpendicular = (current_position['latitude'] + k * (current_position['longitude'] - b)) / (1 + k ** 2)
        y_perpendicular = k * x_perpendicular + b

        # 计算 LOS 点的坐标（目标点的 ratio 倍与垂直点的 (1 - ratio) 倍）
        DotLos = {'latitude': target_point['latitude'] * ratio + x_perpendicular * (1 - ratio),
                  'longitude': target_point['longitude'] * ratio + y_perpendicular * (1 - ratio)}
    else:
        # 如果斜率为无穷大，则目标点与下一个点的连线垂直于 x 轴
        DotLos = {'latitude': target_point['latitude'] * ratio + current_position['latitude'] * (1 - ratio),
                  'longitude': target_point['longitude']}

    # 计算当前位置与 LOS 点的角度
    angle = calculate_angle(current_position['latitude'], current_position['longitude'],
                            DotLos['latitude'], DotLos['longitude'])
    return angle

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371.0
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c

    return distance

def calculate_angle(lat1, lon1, lat2, lon2):

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    angle = math.atan2(y, x)

    angle = math.degrees(angle)

    return angle


# 控制算法
def Model():
    global path_index
    path_index = 1
    ID = 0

    path = [{'longitude': 119.556952728385, 'latitude': 35.403159625348}, {'longitude': 119.55685508445046, 'latitude': 35.40319039848897}, {'longitude': 119.55680571743841, 'latitude': 35.403240605836324}, {'longitude': 119.55685748094588, 'latitude': 35.40331987429862}, {'longitude': 119.55690682018562, 'latitude': 35.40335603349293}, {'longitude': 119.5568112172273, 'latitude': 35.40340097159769}, {'longitude': 119.55670246132496, 'latitude': 35.40341663504651}, {'longitude': 119.55660685836665, 'latitude': 35.40346157315127}, {'longitude': 119.55653593948014, 'latitude': 35.403530365713316}, {'longitude': 119.55644143794608, 'latitude': 35.40357681603784}, {'longitude': 119.55633308041405, 'latitude': 35.40359420655382}, {'longitude': 119.55622795102343, 'latitude': 35.403621725556086}, {'longitude': 119.5561407469449, 'latitude': 35.403676796562294}, {'longitude': 119.55608198627735, 'latitude': 35.40375277719634}, {'longitude': 119.55599916862232, 'latitude': 35.40381214934942}, {'longitude': 119.55589635038022, 'latitude': 35.403844928933914}, {'longitude': 119.55580814748205, 'latitude': 35.4038989398562}, {'longitude': 119.55570252579307, 'latitude': 35.40392518517006}, {'longitude': 119.55561432289488, 'latitude': 35.403979196092344}, {'longitude': 119.55554190968247, 'latitude': 35.40404695337713}, {'longitude': 119.55544533835888, 'latitude': 35.404090503421116}, {'longitude': 119.55535875746708, 'latitude': 35.40414621963547}, {'longitude': 119.55530085402988, 'latitude': 35.40422263340612}, {'longitude': 119.55521427313806, 'latitude': 35.40427834962048}, {'longitude': 119.55511927401605, 'latitude': 35.40432412512628}, {'longitude': 119.55501073299644, 'latitude': 35.404340743258636}, {'longitude': 119.55491573387442, 'latitude': 35.40438651876444}, {'longitude': 119.55484107254829, 'latitude': 35.40445264606157}, {'longitude': 119.55479875280425, 'latitude': 35.40453553103963}, {'longitude': 119.554793879028, 'latitude': 35.404625176547}]

    while True:

        if longitude is None or latitude is None or pitch is None or roll is None or yaw is None:
            time.sleep(0.1)
            continue

        Path_send(path)

        target_yaw = LOS({'latitude': latitude, 'longitude': longitude}, path[path_index], path)  # 假设初始目标点为路径上的第一个点

        steer = PID_controller(yaw, target_yaw)

        if path_index >= len(path) - 1:
            throttle = 0
            steer = 0
            print("抵达终点")
        else:
            throttle = 0.5
        Control_send(throttle, steer)

        if path_index >= len(path) - 1:
            break

        time.sleep(dt)


if __name__ == "__main__":
    # 多线程接收
    thread_GPS = threading.Thread(target=GPS_receive, daemon=True)
    thread_IMU = threading.Thread(target=IMU_receive, daemon=True)
    thread_Buoy = threading.Thread(target=Buoy_receive, daemon=True)
    thread_GPS.start()
    thread_IMU.start()
    thread_Buoy.start()

    # 控制程序
    Model()