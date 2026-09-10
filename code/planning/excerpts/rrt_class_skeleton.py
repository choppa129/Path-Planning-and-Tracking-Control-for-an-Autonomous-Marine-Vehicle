class RRT:
    class Node:
        def __init__(self, x, y):

    class AreaBounds:
        def __init__(self, area):

    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=10.0, goal_sample_rate=5, max_iter=1000,
                 play_area=None, robot_radius=0.0):

    def planning(self, animation=True, camera=None):

    def steer(self, from_node, to_node, extend_length=float("inf")):

    def generate_final_course(self, goal_ind):

    def smooth_path(self, path):

    @staticmethod
    def line_collision_free(node1, node2, obstacle_list):

    def calc_dist_to_goal(self, x, y):

    def sample_free(self):

    def draw_graph(self, rnd=None, camera=None):

    @staticmethod
    def plot_circle(x, y, size, color="-b"):

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):

    @staticmethod
    def is_inside_play_area(node, play_area):

    @staticmethod
    def obstacle_free(node, obstacle_list, robot_radius):

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):

def run_rrt():

def main():
    run_rrt()

if __name__ == '__main__':
    main()
