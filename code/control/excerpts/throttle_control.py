def LOS(current_position, target_point, Delta = 5):

    def calculate_distance(lat1, lon1, lat2, lon2):
    def calculate_angle(lat1, lon1, lat2, lon2):

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

    if distance_to_target < 0.005 or distance_to_target_next < 0.005:
        throttle = 0.4
    if distance_to_target > 0.005 and distance_to_target < 0.02 and distance_to_target_next > 0.005:
        throttle = 0.8
    if distance_to_target > 0.02 and distance_to_target_next > 0.005:
        throttle = 1.0

    if target_point_next['latitude'] != target_point['latitude']:
    else:
    angle = calculate_angle(current_position['latitude'], current_position['longitude'],
                            DotLos['latitude'], DotLos['longitude'])
    return angle, throttle, x_perpendicular, y_perpendicular

