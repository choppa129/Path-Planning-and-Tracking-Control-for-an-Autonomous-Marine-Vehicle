    def steer(self, from_node, to_node, extend_length=float("inf")):
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        max_angle_diff = math.radians(20)
        if from_node.parent is not None:
            parent_angle = math.atan2(from_node.y - from_node.parent.y, from_node.x - from_node.parent.x)
            angle_diff = abs(theta - parent_angle)
            if angle_diff > max_angle_diff:
                theta = parent_angle + max_angle_diff if theta > parent_angle else parent_angle - max_angle_diff