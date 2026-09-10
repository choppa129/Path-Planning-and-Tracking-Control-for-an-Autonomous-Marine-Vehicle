    distance_to_target = calculate_distance(current_position['latitude'], current_position['longitude'],
                                            target_point['latitude'], target_point['longitude'])
    target_point_next = path[path_index - 1]
    distance_to_target_next = calculate_distance(current_position['latitude'], current_position['longitude'], target_point_next['latitude'], target_point_next['longitude'])
    if distance_to_target < 0.005:
        if path_index >= len(path) -1:
            if distance_to_target < 0.0005:
                path_index = 0
        if path_index < len(path) - 1:
            path_index += 1
        target_point = path[path_index]
    target_point_next = path[path_index - 1]
    if distance_to_target < 0.01 or distance_to_target_next < 0.01:
        throttle = 0.4
    if distance_to_target > 0.01 and distance_to_target < 0.04 and distance_to_target_next > 0.01:
        throttle = 0.8
    if distance_to_target > 0.04 and distance_to_target_next > 0.01:
        throttle = 1.0