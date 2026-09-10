import math
import random
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=10.0, goal_sample_rate=5, max_iter=1000,
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

        return valid_path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        max_angle_diff = math.radians(20)
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
            j = i + 1
            while j < len(path):
                if self.line_collision_free(path[i], path[j], self.obstacle_list):
                    pruned_path.append(path[j])
                    i = j
                    break
                j += 1
            i += 1

        return pruned_path

    @staticmethod
    def line_collision_free(node1, node2, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx_list = [ox - x for x in [node1.x, node2.x]]
            dy_list = [oy - y for y in [node1.y, node2.y]]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size + 1) ** 2:
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


def update(frame, rrt, ax, frames):
    valid_path = rrt.planning(animation=True, camera=None)
    if valid_path is not None:
        rrt.draw_graph(camera=ax)
        plt.plot([x for (x, y) in valid_path], [y for (x, y) in valid_path], '-r')
        plt.savefig(f'frame_{frame:03d}.png')


        wrong_path = set((node.x, node.y) for node in rrt.node_list) - set((x, y) for (x, y) in valid_path)
        wrong_path = sorted(rrt.node_list, key=lambda node: rrt.node_list.index(node))
        for i in range(len(wrong_path) - 1):
            plt.plot([wrong_path[i].x, wrong_path[i + 1].x], [wrong_path[i].y, wrong_path[i + 1].y], '--c')
    return []





def main(gx=90, gy=90.0):
    print("start " + __file__)

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
        for point in path:

            lon, lat = convert_to_lon_lat(point[0], point[1])
            Fpath.append({'longitude': lon, 'latitude': lat})
            print(f"    {{'longitude': {lon}, 'latitude': {lat}}},")
        print("]")
        return Fpath
    map_width = 243
    map_height = 167
    min_lon = 119.554793879028
    max_lon = 119.557477854023
    min_lat = 35.403159625348
    max_lat = 35.4046581649303
    Fpath = []
    start_lon, start_lat = 119.554793879028, 35.404625176547
    goal_lon, goal_lat = 119.556952728385, 35.403159625348
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

    print(start_x, start_y, goal_x, goal_y)

    rrt = RRT(
        start=[start_x, start_y],
        goal=[goal_x, goal_y],
        rand_area=[0, 250],
        obstacle_list=obstacle_list,
        play_area=[0, 300, 0, 200],
        robot_radius=4.0
    )

    valid_path = rrt.planning(animation=False, camera=None)

    fig, ax = plt.subplots()

    rrt.draw_graph(rnd=None, camera=ax)
    plt.plot([x for (x, y, _) in rrt.obstacle_list], [y for (x, y, _) in rrt.obstacle_list], 'sk')
    plt.plot(rrt.start.x, rrt.start.y, 'go', label='Start Point')
    plt.plot(rrt.end.x, rrt.end.y, 'rx', label='End Point')

    if valid_path is not None:
        final_path_nodes = [rrt.Node(x, y) for (x, y) in valid_path]
        smoothed_path = rrt.smooth_path(final_path_nodes)
        plt.plot([node.x for node in smoothed_path], [node.y for node in smoothed_path], '-r', label='Smoothed Path')

    if valid_path is not None:
        Fpath = output_path(valid_path)
    print(Fpath)
    plt.grid(True)
    plt.legend()
    plt.savefig('final_result.png')
    plt.show()


if __name__ == '__main__':
    main()

