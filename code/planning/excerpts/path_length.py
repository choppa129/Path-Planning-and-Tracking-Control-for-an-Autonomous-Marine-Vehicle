
def run_rrt():
    # Initialise the map
    start_x, start_y, goal_x, goal_y, obstacle_list = initialize_map()
    start_time = time.time()  # Record the start time
    shortest_path = None
    shortest_length = float('inf')
    path_lengths = []
    path_turnings = []
    path_numbers = []
    range_number = 200
    sum_number = 0

    # Carry out the path planning
    for _ in range(range_number):
        rrt = RRT(
            start=[start_x, start_y],
            goal=[goal_x, goal_y],
            rand_area=[0, 250],
            obstacle_list=obstacle_list,
            play_area=[0, 300, 0, 200],
            robot_radius=4.0
        )

        valid_path = rrt.planning(animation=False, camera=None)

        # If a valid path has been found, update the shortest path
        if valid_path is not None:
            sum_number += 1
            final_path_nodes = [rrt.Node(x, y) for (x, y) in valid_path]
            smoothed_path = rrt.smooth_path(final_path_nodes)
            path_length = calculate_path_length(smoothed_path)
            path_lengths.append(path_length)
            path_turning = calculate_path_turning(smoothed_path) * 180 / math.pi
            path_turnings.append(path_turning)
            path_numbers.append(len(smoothed_path))
            if path_length < shortest_length:
                shortest_length = path_length
                shortest_path = smoothed_path


    # If the shortest path has been found, visualise and output it
    if shortest_path is not None:
        visualize_result(rrt, shortest_path)
    end_time = time.time()  # Record the end time
    total_time = end_time - start_time  # Compute the total time

    # Write the data to an Excel sheet
    data = {
        'Path Length': path_lengths,
        'Path Turning': path_turnings,
        'Path Number': path_numbers,
    }
    df = pd.DataFrame(data)
    df.to_excel('path_data.xlsx', index=False)
