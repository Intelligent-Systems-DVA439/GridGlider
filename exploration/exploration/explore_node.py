import math

import rclpy
from rclpy.node import Node

from .classes.structs import ParametersInfo
from .navigation.control_tb3 import Tb3Control


class Explore(Node):
    """
    Initiate all the nodes for subscription
    """

    def __init__(self):
        super().__init__("Explore")
        # Declare parameters with a default value if not provided by the YAML configuration
        self.parameters_info = ParametersInfo()
        self.declare_parameter("speed", 0.1)
        self.declare_parameter("max_steering_angle", math.radians(30))
        self.declare_parameter("near_obstacle", 0.27)
        self.declare_parameter("lookahead_distance", 0.34)
        self.declare_parameter("prioritize_direction", False)
        self.declare_parameter("distance_to_waypoint_threshold", 3.0)
        self.declare_parameter("control_loop_period", 0.1)
        self.declare_parameter("non_progress_threshold", 3)
        self.declare_parameter("progress_timeout_seconds", 5.0)

        self.parameters_info.progress_timeout_seconds = (
            self.get_parameter("progress_timeout_seconds")
            .get_parameter_value()
            .double_value
        )
        self.parameters_info.non_progress_threshold = (
            self.get_parameter("non_progress_threshold")
            .get_parameter_value()
            .integer_value
        )
        self.parameters_info.speed = (
            self.get_parameter("speed").get_parameter_value().double_value
        )

        angle = (
            self.get_parameter("max_steering_angle").get_parameter_value().double_value
        )

        self.parameters_info.max_steering_angle = math.radians(angle)

        self.parameters_info.distance_to_waypoint_threshold = (
            self.get_parameter("distance_to_waypoint_threshold")
            .get_parameter_value()
            .double_value
        )
        self.parameters_info.lookahead_distance = (
            self.get_parameter("lookahead_distance").get_parameter_value().double_value
        )
        self.parameters_info.prioritize_direction = (
            self.get_parameter("prioritize_direction").get_parameter_value().bool_value
        )
        self.parameters_info.near_obstacle = (
            self.get_parameter("near_obstacle").get_parameter_value().double_value
        )
        self.parameters_info.control_loop_period = (
            self.get_parameter("control_loop_period").get_parameter_value().double_value
        )
        # Print the values
        self.get_logger().info(f"Speed: {self.parameters_info.speed}")
        self.get_logger().info(
            f"Max Steering Angle: {self.parameters_info.max_steering_angle}"
        )
        self.get_logger().info(f"Near Obstacle: {self.parameters_info.near_obstacle}")
        self.get_logger().info(
            f"Lookahead Distance: {self.parameters_info.lookahead_distance}"
        )
        self.get_logger().info(
            f"Prioritize Direction: {self.parameters_info.prioritize_direction}"
        )
        self.get_logger().info(
            f"Distance to Waypoint Threshold: {self.parameters_info.distance_to_waypoint_threshold}"
        )
        self.get_logger().info(
            f"Control Loop Period: {self.parameters_info.control_loop_period}"
        )
        self.fuzzy_navigation = Tb3Control(self)


def main(args=None):
    """
    Main function which starts the programs and the nodes in a multithreaded environment
    """
    rclpy.init(args=args)
    main_node = Explore()

    try:
        rclpy.spin(main_node)
    finally:
        main_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
