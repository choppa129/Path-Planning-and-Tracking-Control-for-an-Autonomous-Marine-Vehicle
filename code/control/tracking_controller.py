import socket
import json
import threading
import time
import pandas as pd
import math

Host = '127.0.0.1'  # IP address

Port_GPS = 60100  # GPS port
Port_IMU = 60101  # IMU port
Port_Buoy = 60103  # buoy port

Port_Control = 60199  # control port
Port_Path = 60198  # path-output port

socket_GPS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create the UDP sockets
socket_IMU = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_Buoy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

socket_Control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_Path = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

socket_GPS.bind((Host, Port_GPS))  # Bind the ports
socket_IMU.bind((Host, Port_IMU))
socket_Buoy.bind((Host, Port_Buoy))

longitude = None  # longitude
latitude = None  # latitude

pitch = None  # pitch angle
roll = None  # roll angle
yaw = None  # yaw angle
velocity = None  # velocity

buoy_list = None  # buoy positions

path = None  # global path variable

pre_error = 0
sum_error = 0

def GPS_receive():
    global longitude, latitude
    while True:
        data, _ = socket_GPS.recvfrom(1024)
        data_json = json.loads(data.decode('utf-8'))
        longitude = data_json["longitude"]
        latitude = data_json["latitude"]

def IMU_receive():
    global pitch, roll, yaw, velocity
    while True:
        data, _ = socket_IMU.recvfrom(1024)
        data_json = json.loads(data.decode('utf-8'))
        pitch = data_json["pitch"]
        roll = data_json["roll"]
        yaw = data_json["yaw"]
        velocity = data_json["velocity"]

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

def Path_planning():
    global path
    path = [{'longitude': 119.555323713895, 'latitude': 35.4059490670712}, {'longitude': 119.554793879028, 'latitude': 35.404625176547}, {'longitude': 119.5561758920993, 'latitude': 35.40428068759785}, {'longitude': 119.55642159573607, 'latitude': 35.40409235970548}, {'longitude': 119.55656392206485, 'latitude': 35.40389586868287}, {'longitude': 119.556952728385, 'latitude': 35.403159625348}]

def Path_send(path_points):
    path = {"points": path_points}
    path = json.dumps(path)
    path = path.encode('utf-8')
    socket_Path.sendto(path, (Host, Port_Path))

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

def Model():
    global path
    global path_index
    path_index = 0
    total_distance = 0  # Total distance travelled by the ship
    total_angle = 0  # Total angle swept by the bow of the ship
    prev_yaw = yaw
    perpendicular_distances = []  # Length of the perpendicular from the hull to the path
    start_time = time.time()  # Time at which the ship started
    end_time = None  # Time at which the ship reached the goal
    prev_position = {'latitude': None, 'longitude': None}  # Previous position of the hull
    angles = []
    distances = []
    indexs = []
    Path_send(path)

    while True:
        if longitude is None or latitude is None or pitch is None or roll is None or yaw is None:
            time.sleep(0.1)
            continue

        target_yaw, throttle, x_per, y_per = LOS({'latitude': latitude, 'longitude': longitude}, path[path_index])

        steer = PID_controller(yaw, target_yaw, throttle)

        if path_index >= len(path) :
            throttle = 0
            steer = 0
            print("Goal point reached")
            Control_send(throttle, steer)
            end_time = time.time()  # Time at which the ship reached the goal
            break  # Leave the loop

        Control_send(throttle, steer)


        # Compute the distance travelled by the ship and add it to the total length
        if prev_position['latitude'] is not None and prev_position['longitude'] is not None:
            distance = calculate_distance(prev_position['latitude'], prev_position['longitude'], latitude, longitude)
            if path_index > 1:
                total_distance += distance
                distances.append(distance)

        # Compute the heading angle of the ship and add it to the total angle
        if prev_yaw is not None:
            angle = abs(prev_yaw - yaw)
            if path_index > 1:
                total_angle += abs(angle)
                angles.append(angle)

        # Compute the length of the perpendicular from the hull to the path and store it

        perpendicular_distance = calculate_distance(latitude, longitude, x_per, y_per)
        if perpendicular_distance is not None:
            if path_index > 1:
                perpendicular_distances.append(perpendicular_distance)
                indexs.append(path_index + 1)

        prev_position['latitude'] = latitude
        prev_position['longitude'] = longitude
        prev_yaw = yaw
        #print(latitude, longitude)
        time.sleep(0.1)
        #print(total_distance)
    data = {
         'Path_Index:': indexs,
         #'Distances': distances,
         #'Angles': angles,
         'Perpendicular_distances': perpendicular_distances,
    }
    df = pd.DataFrame(data)
    df.to_excel('path_data.xlsx', index=False)
    print(len(angles), len(perpendicular_distances), len(distances), len(indexs))
    print("Total distance travelled by the ship:", total_distance * 1000, "m")
    print("Total angle swept by the bow of the ship:", total_angle, "deg")
    print("Mean length of the perpendicular from the hull to the path:", 1000 * sum(perpendicular_distances)/len(perpendicular_distances))

    # Compute the time the ship needs to travel from the start to the goal
    if end_time is not None:
        time_taken = end_time - start_time
        print("Time from the start point to the goal point:", time_taken, "s")
    else:
        print("The travel time cannot be computed because the goal point was not reached")

def perpendicular_distance_to_path(position, point1, point2):
    # Compute the perpendicular distance from the position of the hull to the path
    lat1, lon1 = position['latitude'], position['longitude']
    lat2, lon2 = point1['latitude'], point1['longitude']
    lat3, lon3 = point2['latitude'], point2['longitude']

    # Length of the segment
    line_length = calculate_distance(lat2, lon2, lat3, lon3)

    # If the segment has zero length, return the point-to-point distance
    if line_length == 0:
        return calculate_distance(lat1, lon1, lat2, lon2)

    # Compute the unit vector of the segment
    dx = (lon3 - lon2) / line_length
    dy = (lat3 - lat2) / line_length

    # Compute the coordinates of the perpendicular distance
    x = (lon1 - lon2) * dx + (lat1 - lat2) * dy
    y = (lat1 - lat2) * dx - (lon1 - lon2) * dy

    # The length of the perpendicular from the hull to the path is the absolute value of y
    return abs(y)

def LOS(current_position, target_point, Delta = 5):
    global path_index
    global sum_error
    global pre_error
    distance_per_degree_lat = 128320
    distance_per_degree_lon = 100000

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

    distance_to_target = calculate_distance(current_position['latitude'], current_position['longitude'],
                                            target_point['latitude'], target_point['longitude'])
    target_point_next = path[path_index - 1]
    distance_to_target_next = calculate_distance(current_position['latitude'], current_position['longitude'], target_point_next['latitude'], target_point_next['longitude'])
    if distance_to_target < 0.005:
        if path_index >= len(path) -1:
            if distance_to_target < 0.0035:
                path_index += 1
                return 0, 0, current_position['latitude'], current_position['longitude']
        if path_index < len(path) - 1:
            path_index += 1
            target_point = path[path_index]
            target_point_next = path[path_index - 1]
    #print(path_index)
    if distance_to_target < 0.005 or distance_to_target_next < 0.005:
        throttle = 0.4
    if distance_to_target > 0.005 and distance_to_target < 0.02 and distance_to_target_next > 0.005:
        throttle = 0.8
    if distance_to_target > 0.02 and distance_to_target_next > 0.005:
        throttle = 1.0
    #throttle = 1

    #print("target_point:", target_point)
    #print("target_point_next:", target_point_next)
    if target_point_next['latitude'] != target_point['latitude']:
        k = (target_point_next['longitude'] - target_point['longitude']) / (target_point_next['latitude'] - target_point['latitude'])
        b = target_point_next['longitude'] - k * target_point_next['latitude']
        x_perpendicular = (current_position['latitude'] + k * (current_position['longitude'] - b)) / (1 + k ** 2)
        y_perpendicular = k * x_perpendicular + b
        direction_angle = calculate_angle(x_perpendicular, y_perpendicular, target_point['latitude'], target_point['longitude'])
        delta_latitude = (Delta / distance_per_degree_lat) * math.cos(math.radians(direction_angle))
        delta_longitude = (Delta / distance_per_degree_lon) * math.sin(math.radians(direction_angle))
        #print("x_perpendicular:" + str(x_perpendicular) + " y_perpendicular:" + str(y_perpendicular))
        #print("delta_latitude:" + str(delta_latitude) + " delta_longitude:" + str(delta_longitude))

        new_latitude = x_perpendicular + delta_latitude
        new_longitude = y_perpendicular + delta_longitude
        DotLos = {'latitude': new_latitude, 'longitude':new_longitude}
    else:
        DotLos = {'latitude': target_point['latitude'],
                  'longitude': target_point['longitude']}

    angle = calculate_angle(current_position['latitude'], current_position['longitude'],
                            DotLos['latitude'], DotLos['longitude'])
    #print(DotLos)
    #print(angle)
    return angle, throttle, x_perpendicular, y_perpendicular

def PID_controller(current_yaw, target_yaw, throttle):
    global pre_error, sum_error
    yaw_difference = target_yaw - current_yaw

    if abs(yaw_difference) > 180:
        if yaw_difference > 0:
            error = yaw_difference - 360
        else:
            error = yaw_difference + 360
    else:
        error = yaw_difference

    sum_error += error * 0.1
    d_error = error - pre_error
    Kp = 0.15 * math.sqrt(throttle)
    Ki = 0
    Kd = 0 * math.sqrt(throttle)
    steer = Kp * error + Ki * sum_error + Kd * d_error

    steer = max(min(steer, 1), -1)

    pre_error = error
    #print(error)
    #print('steer: ', steer)

    return steer

def Control_send(throttle, steer):
    control_cmd = {}
    control_cmd["throttle"] = throttle
    control_cmd["steer"] = steer
    #print(control_cmd)
    control_cmd = json.dumps(control_cmd)
    control_cmd = control_cmd.encode('utf-8')
    socket_Control.sendto(control_cmd, (Host, Port_Control))

if __name__ == "__main__":
    # Multi-threaded reception
    thread_GPS = threading.Thread(target=GPS_receive, daemon=True)
    thread_IMU = threading.Thread(target=IMU_receive, daemon=True)
    thread_Buoy = threading.Thread(target=Buoy_receive, daemon=True)
    thread_GPS.start()
    thread_IMU.start()
    thread_Buoy.start()

    # Path planning
    Path_planning()

    # Control program
    Model()
