import math

import numpy as np
from cluster_interfaces.msg import AstarPath
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.logging import get_logger
from rclpy.timer import Timer
from std_msgs.msg import String

from exploration.classes.marker_class import MarkerPublish
from exploration.classes.sensor_class import SensorRead
from exploration.classes.structs import OccupancyGridInfo
from exploration.fuzzy.fuzzy_utils import (create_fuzzy_system,
                                           fuzzy_movement_choice)
from exploration.navigation.path_planning import (enhanced_pure_pursuit,
                                                  near_obstacle)
from exploration.utility.helper_functions import \
    calculate_direction_and_distance


class Tb3Control:
    def __init__(self, parent_node):
        self.node = parent_node
        self.logger = get_logger("TB3Control")

        self.sensor_data = SensorRead(self.node)
        self.marker_to_publish = MarkerPublish(self.node)
        self.fuzzy_system = create_fuzzy_system("centroid", 0)

        # Publisher to control the velocity and angular velocity of tb3
        self.control_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.astar_path_subscription = self.node.create_subscription(
            AstarPath, "/astar_path", self.astar_path_callback, 10
        )
        self.trigger_astar_publisher = self.node.create_publisher(
            String, "/astar_update", 10
        )

        self.control_timer = self.node.create_timer(
            self.node.parameters_info.control_loop_period, self.control_loop
        )

        self.costmap_subscription = self.node.create_subscription(
            OccupancyGrid, "/costmap", self.unpack_costmap, 10
        )
        self.grid_path = []  # Wayponts provided by A*
        self.world_path = []
        self.backup_world_path = []
        self.current_waypoint_index = 0
        self.last_orientation_difference = float("inf")
        self.costmap = OccupancyGridInfo()

        self.last_waypoint_distance = float("inf")  # Initialize with infinity
        self.progress_timeout_seconds = 10  # Timeout duration in seconds
        self.progress_timer: Timer = None  # Timer for checking progress
        self.initialize_progress_timer()
        # Initialize progress timer and immediately trigger path calculation
        self.non_progress_count = 0  # Initialize a counter for non-progress checks
        self.mode = "normal"

        # Use a lambda or a dedicated method to delay the initial path request slightly
        self.one_shot_timer = self.node.create_timer(
            3, lambda: self.trigger_astar_once_init()
        )

    def trigger_astar_once_init(self):
        # This method wraps the original trigger_astar call and cancels the timer after execution
        self.logger.info("Init Astar")
        self.trigger_astar()
        self.one_shot_timer.cancel()  # Cancel the timer to prevent future executions

    def unpack_costmap(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.costmap.resolution = msg.info.resolution
        self.costmap.originX = msg.info.origin.position.x
        self.costmap.originY = msg.info.origin.position.y
        self.costmap.data = np.array(msg.data).reshape(height, width)

    def astar_path_callback(self, msg):
        # self.logger.info("Got Astar path")
        self.logger.info(
            f"Received new A* path with {len(msg.world_path_x)} waypoints."
        )
        if len(msg.world_path_x) <= 5:
            self.mode = "fuzzy"
            self.trigger_astar()
        else:
            self.mode = "normal"
            self.world_path = []
            self.grid_path = []
            self.world_path = list(zip(msg.world_path_x, msg.world_path_y))
            self.grid_path = list(zip(msg.grid_path_x, msg.grid_path_y))
            self.backup_world_path = self.world_path
            self.current_waypoint_index = 0
            self.last_waypoint_distance = float("inf")

    def control_loop(self):
        """
        Periodically called function to compute and publish control commands based on fuzzy logic.
        """
        # self.logger.info("Entering control loop")

        if not self.world_path:
            self.logger.info("No path available")
            self.mode = "fuzzy"
        if self.sensor_data.lidar_data is None:
            self.logger.info("No sensor data..")
            return

        if near_obstacle(
            self.sensor_data.lidar_data,
            self.node.parameters_info.near_obstacle or self.mode == "fuzzy",
        ):
            if self.mode != "fuzzy":
                self.mode = "fuzzy"
            # self.logger.info("Near an obstacle, switching to fuzzy navigation.")
            self.fuzzy_navigation()
            self.trigger_astar()

        if self.mode == "normal":
            # self.logger.info("Following path using pure pursuit control.")

            linear_velocity, angular_velocity, index = enhanced_pure_pursuit(
                self.sensor_data.odom.x,
                self.sensor_data.odom.y,
                self.sensor_data.odom.yaw,
                self.world_path,
                self.current_waypoint_index,
                self.node.parameters_info.lookahead_distance,
                self.node.parameters_info.speed,
                self.node.parameters_info.max_steering_angle,
            )

            self.update_velocity(linear_velocity, angular_velocity)
        self.has_reached_goal()

    def fuzzy_navigation(self):
        # Define what a single fuzzy command does
        np_sensor_data = np.array(self.sensor_data.lidar_data)
        np_sensor_data[np_sensor_data == math.inf] = 3.5
        linear_velocity, angular_velocity = fuzzy_movement_choice(
            np_sensor_data, self.fuzzy_system
        )
        self.update_velocity(linear_velocity, angular_velocity)

    def has_reached_goal(self):
        if self.current_waypoint_index < len(self.world_path):
            current_waypoint = self.world_path[self.current_waypoint_index]
            direction_to_waypoint, distance_to_waypoint = (
                calculate_direction_and_distance(
                    self.sensor_data.odom, current_waypoint
                )
            )
        else:
            self.trigger_astar()
            if self.mode == "normal":
                self.mode = "fuzzy"
            return

        # Check if the robot is close to the current waypoint
        if (
            distance_to_waypoint
            < self.node.parameters_info.distance_to_waypoint_threshold
        ):
            self.current_waypoint_index += 1  # Move to the next waypoint
            point_array = Point()
            point_array.x = current_waypoint[0]
            point_array.y = current_waypoint[1]
            self.marker_to_publish.publish_target_marker(
                point_array, self.sensor_data.occupancy_grid
            )

            if self.current_waypoint_index >= len(self.world_path):
                self.logger.info("Completed all waypoints in the path.")
                self.trigger_astar()

                if self.mode == "normal":
                    self.mode = "fuzzy"
                return

    def update_velocity(self, speed, direction):
        self.logger.debug(f"V={speed:.2f}, W={direction:.2f}")

        # Create and publish a Twist message to update the robot's velocity
        odom_msg = Twist()
        odom_msg.linear.x = speed
        odom_msg.angular.z = direction
        self.control_publisher.publish(odom_msg)

    def trigger_astar(self):
        msg = String()
        msg.data = "Need new path"
        self.trigger_astar_publisher.publish(msg)

    def initialize_progress_timer(self):
        # Creates a timer that checks for progress every `progress_timeout_seconds`
        self.progress_timer = self.node.create_timer(
            self.node.parameters_info.progress_timeout_seconds, self.check_progress
        )

    def check_progress(self):
        if (
            self.world_path
            and self.current_waypoint_index is not None
            and self.current_waypoint_index < len(self.world_path)
        ):
            current_waypoint_world = self.world_path[self.current_waypoint_index]
            bearing_to_waypoint, distance_to_waypoint = (
                calculate_direction_and_distance(
                    self.sensor_data.odom, current_waypoint_world
                )
            )

            orientation_difference = abs(
                self.sensor_data.odom.yaw - bearing_to_waypoint
            )
            orientation_difference = min(
                orientation_difference, 2 * math.pi - orientation_difference
            )

            # Determine if the robot has made any progress towards the waypoint
            progress_made = self.evaluate_progress(
                distance_to_waypoint, orientation_difference
            )

            if progress_made:
                self.last_waypoint_distance = distance_to_waypoint
                self.logger.info("Progress detected, continuing navigation.")
                self.non_progress_count = 0
            else:
                self.handle_lack_of_progress()

    def evaluate_progress(self, distance_to_waypoint, orientation_difference):
        # Criteria for progress: distance decreases or orientation aligns over time
        distance_improving = distance_to_waypoint < self.last_waypoint_distance
        orientation_improving = (
            orientation_difference < self.last_orientation_difference
        )

        self.last_orientation_difference = orientation_difference

        return distance_improving or orientation_improving

    def handle_lack_of_progress(self):
        self.non_progress_count += 1
        if self.non_progress_count > self.node.parameters_info.non_progress_threshold:
            self.logger.info("No progress made towards waypoint, resetting path...")
            self.trigger_astar()
            self.mode = "fuzzy" if self.mode == "normal" else self.mode
            self.non_progress_count = 0
