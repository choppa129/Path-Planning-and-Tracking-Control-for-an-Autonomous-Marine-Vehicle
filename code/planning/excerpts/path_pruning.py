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
