import math
import random
import os
import matplotlib.pyplot as plt
import pandas as pd
import time

map_width = 243
map_height = 167
min_lon = 119.554793879028
max_lon = 119.557477854023
min_lat = 35.403159625348
max_lat = 35.4046581649303


class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=10, goal_sample_rate=5, max_iter=1000,
                 play_area=None, robot_radius=0.0):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius
        self.some_extra_radius = 3.0

    def planning(self, animation=True, camera=None):
        self.node_list = [self.start]
        valid_path = None

        for i in range(self.max_iter):
            rnd_node = self.sample_free()

            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.is_inside_play_area(new_node, self.play_area) and self.obstacle_free(new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= (
                    self.expand_dis + self.some_extra_radius):
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.obstacle_free(final_node, self.obstacle_list, self.robot_radius):
                    valid_path = self.generate_final_course(len(self.node_list) - 1)
                    break

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node, camera)

        if valid_path is not None:
            final_path_nodes = [self.Node(x, y) for (x, y) in valid_path]
            smoothed_path = self.smooth_path(final_path_nodes)
            valid_path = [[node.x, node.y] for node in smoothed_path]

        return valid_path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        max_angle_diff = math.radians(30)
        if from_node.parent is not None:
            parent_angle = math.atan2(from_node.y - from_node.parent.y, from_node.x - from_node.parent.x)
            angle_diff = abs(theta - parent_angle)
            if angle_diff > max_angle_diff:
                theta = parent_angle + max_angle_diff if theta > parent_angle else parent_angle - max_angle_diff

        if extend_length >= d:
            new_x = to_node.x
            new_y = to_node.y
        else:
            new_x = from_node.x + math.cos(theta) * extend_length
            new_y = from_node.y + math.sin(theta) * extend_length

        new_node = self.Node(new_x, new_y)
        new_node.path_x = [from_node.x]
        new_node.path_y = [from_node.y]
        new_node.path_x.append(new_x)
        new_node.path_y.append(new_y)

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def smooth_path(self, path):
        pruned_path = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Look for the next pruning point
            j = i + 1
            while j < len(path) and self.line_collision_free(pruned_path[-1], path[j], self.obstacle_list):
                j += 1
            pruned_path.append(path[j-1])
            i += 1

        # Append the goal point to the smoothed path
        pruned_path.append(path[-1])
        unique_path = [pruned_path[-1]]
        for point in reversed(pruned_path[:-1]):
            if point not in unique_path:
                unique_path.insert(0, point)

        return unique_path
    @staticmethod
    def line_collision_free(node1, node2, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            # Compute the relative position of the path segment with respect to the circle
            dx = node2.x - node1.x
            dy = node2.y - node1.y
            d = math.sqrt(dx ** 2 + dy ** 2) + 10 ** (-8)
            t = ((ox - node1.x) * dx + (oy - node1.y) * dy) / (d ** 2)
            t = max(0, min(1, t))  # Make sure that t lies within [0, 1]

            # Compute the point of the path segment closest to the centre of the circle
            closest_x = node1.x + t * dx
            closest_y = node1.y + t * dy

            # The segment intersects the circle if the closest point is within the radius
            dist = math.sqrt((closest_x - ox) ** 2 + (closest_y - oy) ** 2)
            if dist <= size:
                return False

        return True

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def sample_free(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand)
            )
        else:
            rnd = self.Node(self.end.x, self.end.y)

        return rnd

    def draw_graph(self, rnd=None, camera=None):
        if camera is None:
            plt.clf()

        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax, self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin, self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin], "-k")

        all_x = [node.x for node in self.node_list]
        all_y = [node.y for node in self.node_list]
        all_x.extend([ox for (ox, oy, size) in self.obstacle_list])
        all_y.extend([oy for (ox, oy, size) in self.obstacle_list])

        if self.play_area is not None:
            all_x.extend([self.play_area.xmin, self.play_area.xmax])
            all_y.extend([self.play_area.ymin, self.play_area.ymax])

        plt.axis("equal")
        plt.axis([min(all_x) - 2, max(all_x) + 2, min(all_y) - 2, max(all_y) + 2])

        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def is_inside_play_area(node, play_area):
        if play_area is None:
            return True

        if node.x < play_area.xmin or node.x > play_area.xmax or node.y < play_area.ymin or node.y > play_area.ymax:
            return False
        else:
            return True

    @staticmethod
    def obstacle_free(node, obstacle_list, robot_radius):
        if node is None:
            return False

        for (ox, oy, size) in obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size + robot_radius) ** 2:
                return False

        return True

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

def visualize_result(rrt, shortest_path):
    fig, ax = plt.subplots()
    #rrt.draw_graph(rnd=None, camera=ax)
    plt.plot([x for (x, y, _) in rrt.obstacle_list], [y for (x, y, _) in rrt.obstacle_list], 'sk')
    for (ox, oy, size) in rrt.obstacle_list:
        rrt.plot_circle(ox, oy, size)
    plt.plot(rrt.start.x, rrt.start.y, 'go', label='Start Point')
    plt.plot(rrt.end.x, rrt.end.y, 'rx', label='End Point')
    # Make sure that the start point is contained in the final path
    plt.plot([node.x for node in shortest_path], [node.y for node in shortest_path], '-r', label='Smoothed Path')
    plt.grid(True)
    plt.legend()
    plt.savefig('final_result_shortest_path.png')
    plt.show()

    # Output the path coordinates
    Fpath = output_path(shortest_path)
    print(Fpath)

def calculate_path_turning(path):
    total_turning = 0.0
    for i in range(1, len(path) - 1):
        prev_node = path[i - 1]
        current_node = path[i]
        next_node = path[i + 1]

        angle1 = math.atan2(current_node.y - prev_node.y, current_node.x - prev_node.x)
        angle2 = math.atan2(next_node.y - current_node.y, next_node.x - current_node.x)

        turning = abs(angle2 - angle1)
        total_turning += turning

    return total_turning


def run_rrt():
    # Initialise the map
    start_x, start_y, goal_x, goal_y, obstacle_list = initialize_map()
    start_time = time.time()  # Record the start time
    shortest_path = None
    shortest_length = float('inf')
    path_lengths = []
    path_turnings = []
    path_numbers = []
    path_score = []
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
            Path_score = 0.47 * ((358.78 - path_length) / 104.07) + 0.28 * ((1500 - path_turning) /1500) + 0.18 * ((42 - len(smoothed_path)) / 40)
            path_score.append(Path_score)
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
        'Path Score': path_score,
    }
    df = pd.DataFrame(data)
    df.to_excel('path_data.xlsx', index=False)

    # Output the average values
    avg_path_length = sum(path_lengths) / sum_number
    avg_path_turning = sum(path_turnings) / sum_number
    avg_path_number = sum(path_numbers) / sum_number
    avg_Fpath_length = shortest_length
    avg_Fpath_turning = calculate_path_turning(shortest_path) * 180 / math.pi
    avg_Fpath_number = len(shortest_path)
    Fpath_score = 0.47 * ((358.78 - avg_Fpath_length) / 104.07) + 0.28 * ((1500 - avg_Fpath_turning) / 1500) + 0.18 * (
                (42 - avg_Fpath_number)/ 40) + 0.07 * ((15 - total_time) / 15)
    print("Average Path Length:", avg_path_length)
    print("Average Path Turning:", avg_path_turning)
    print("Average Path Number:", avg_path_number)
    print("Fpath Length:", avg_Fpath_length)
    print("Fpath Turning:", avg_Fpath_turning)
    print("Fpath Number:", avg_Fpath_number)
    print("Totle Time:", total_time)
    print("Fpath Score:", Fpath_score)


def initialize_map():

    goal_lon, goal_lat = 119.554793879028, 35.404625176547
    start_lon, start_lat = 119.556952728385, 35.403159625348
    obstacle_list_lon_lat = [
        [119.555018460937, 35.4042015549788, 7.5],
        [119.555129981354, 35.404018856646, 9.0],
        [119.555436799912, 35.404270055301, 7.5],
        [119.555715656054, 35.40404166038, 7.5],
        [119.555435148977, 35.4046581649303, 4.5],
        [119.555966549108, 35.4044983605518, 4.5],
        [119.556580076699, 35.4046138199964, 15.0],
        [119.555717527382, 35.4036580568152, 15.0],
        [119.557477854023, 35.4045413532812, 15.0],
        [119.556351640929, 35.4038734732142, 15.0],
        [119.556803007337, 35.4041573899354, 15.0],
        [119.555404873774, 35.4034083908053, 7.5],
        [119.556226139639, 35.4034940164831, 7.5],
        [119.557103550729, 35.4036589584682, 7.5],
        [119.557642987829, 35.4040663565401, 7.5],
        [119.557266482617, 35.40425743656, 4.5]
    ]

    start_x, start_y = convert_to_map_coordinate(start_lon, start_lat)
    goal_x, goal_y = convert_to_map_coordinate(goal_lon, goal_lat)
    obstacle_list = [[convert_to_map_coordinate(lon, lat)[0], convert_to_map_coordinate(lon, lat)[1], size] for lon,
                     lat, size in obstacle_list_lon_lat]

    #print(start_x, start_y, goal_x, goal_y)

    return start_x, start_y, goal_x, goal_y, obstacle_list

def convert_to_map_coordinate(lon, lat):
    x = (lon - min_lon) / (max_lon - min_lon) * map_width
    y = (lat - min_lat) / (max_lat - min_lat) * map_height
    return x, y

def convert_to_lon_lat(x, y):
    lon = x / map_width * (max_lon - min_lon) + min_lon
    lat = y / map_height * (max_lat - min_lat) + min_lat
    return lon, lat

def output_path(path):
    print("path = [")
    Fpath = []
    for node in path:
        lon, lat = convert_to_lon_lat(node.x, node.y)
        Fpath.append({'longitude': lon, 'latitude': lat})
        print(f"    {{'longitude': {lon}, 'latitude': {lat}}},")
    print("]")
    return Fpath

def calculate_path_length(path):
    total_length = 0.0
    for i in range(len(path) - 1):
        node1 = path[i]
        node2 = path[i + 1]
        length = math.hypot(node2.x - node1.x, node2.y - node1.y)
        total_length += length
    return total_length

def main():
    run_rrt()

if __name__ == '__main__':
    main()
