class ClusterClass:
    def __init__(self):
        # Initialize storage for cluster information
        self.cluster_labels = []
        self.cluster_centroids = []
        self.cluster_sizes = []


class ParametersInfo:
    def __init__(self) -> None:
        self.offset_to_the_boundary_points = None
        self.lookahead_distance = None
        self.speed = None
        self.occupied_threshold = None
        self.prioritize_direction = None
        self.near_obstacle = None
        self.distance_to_waypoint_threshold = None
        self.control_loop_period = None
        self.max_steering_angle = None
        self.cluster_size = None
        self.eps = None
        self.distance_to_cluster_threshold = None
        self.non_progress_threshold = None
        self.computation_frequency = None
        self.progress_timeout_seconds = None
        self.publish_period = None
        self.use_furthest_point = None


class Range:
    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val

    def __repr__(self):
        return f"({self.min_val}, {self.max_val})"


class SensorInfo:
    """
    Data structure
    """

    def __init__(self) -> None:
        self.velocity_range = Range(-0.26, 0.26)  # Renamed for clarity
        self.angular_range = Range(-1.82, 1.82)  # Renamed for clarity
        self.lidar_range = Range(0, 3.5)


class OccupancyGridInfo:
    def __init__(self) -> None:
        self.msg = None
        self.resolution = None
        self.width = None
        self.height = None
        self.originX = None
        self.originY = None
        self.data = None


class OdomInfo:
    def __init__(self) -> None:
        self.current_velocity = None
        self.current_angular_velocity = None
        self.data = None
        self.x = None
        self.yaw = 0.0
        self.y = None
