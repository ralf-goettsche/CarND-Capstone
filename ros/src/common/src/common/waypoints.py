
import numpy as np
from scipy.spatial import KDTree

class Waypoints:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_2d = [(wp.pose.pose.position.x, wp.pose.pose.position.y)
                             for wp in waypoints]
        self.waypoints_tree = KDTree(self.waypoints_2d)

    def find_closest(self, x, y):
        _, idx = self.waypoints_tree.query([x, y])
        return idx

    def find_closest_ahead(self, x, y):
        closest_idx = self.find_closest(x, y)
        closest_waypoint = np.array(self.waypoints_2d[closest_idx])
        prev_waypoint = np.array(self.waypoints_2d[closest_idx-1])

        path_vec = closest_waypoint - prev_waypoint
        position_vec = closest_waypoint - np.array([x, y])

        projection = np.dot(position_vec, path_vec)
        if projection < 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints)

        return closest_idx

    def get_waypoints_in_range(self, first_idx, last_idx):
        return self.waypoints[first_idx:last_idx]
