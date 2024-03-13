import numpy as np
import rclpy
from cluster_interfaces.msg import AstarPath
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .classes.marker_class import MarkerPublish
from .classes.structs import OccupancyGridInfo, OdomInfo
from .navigation.path_planning import astar
from .utility.helper_functions import convert_world_to_grid_index


class Astar(Node):
    def __init__(self):
        super().__init__("astar")
        # Initalize structs
        # Used to trigger calculations when new map data is received
        self.astar_subscription = self.create_subscription(
            String, "/astar_update", self.compute_astar, 10  # Setup publisher in nav
        )

        self.robot_pose = self.create_subscription(
            Odometry, "/odom", self.robot_pose_callback, 10
        )

        # Used to message the draw node for visualization
        self.publish_astar = self.create_publisher(AstarPath, "/astar_path", 10)

        self.occupancy_grid_costmap = self.create_subscription(
            OccupancyGrid, "/costmap", self.unpack_costmap, 10
        )

        self.frontier_goal_world_subscription = self.create_subscription(
            Point, "/frontier_goal_world", self.frontier_goal, 10
        )

        self.costmap = OccupancyGridInfo()
        self.robot_pose = OdomInfo()
        self.marker_to_publish = MarkerPublish(self)
        self.frontier_goal_world = None
        self.world_path = []
        self.grid_path = []

    def robot_pose_callback(self, msg):
        """
        Get the quaternion position in free space
        """
        self.robot_pose.current_velocity = msg.twist.twist.linear.x
        self.robot_pose.data = msg
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y

    def frontier_goal(self, msg):
        self.frontier_goal_world = msg

    def unpack_costmap(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.costmap.resolution = msg.info.resolution
        self.costmap.originX = msg.info.origin.position.x
        self.costmap.originY = msg.info.origin.position.y
        self.costmap.data = np.array(msg.data).reshape(height, width)
        # self.compute_astar(self.costmap)

    def compute_astar(self, msg):
        if self.costmap.data is None or self.frontier_goal_world is None:
            self.get_logger().info("Waiting for data...")
            return

        self.get_logger().info("Computing astar")
        coordinates = (self.frontier_goal_world.x, self.frontier_goal_world.y)
        goal = convert_world_to_grid_index(
            (coordinates),
            (
                self.costmap.originX,
                self.costmap.originY,
            ),
            self.costmap.resolution,
        )

        # self.logger.info(f"goal_grid: {goal_grid}")
        start = convert_world_to_grid_index(
            [self.robot_pose.x, self.robot_pose.y],
            [
                self.costmap.originX,
                self.costmap.originY,
            ],
            self.costmap.resolution,
        )

        self.grid_path = astar(self.costmap.data, start, goal)
        if self.grid_path is None or self.grid_path is False:
            self.get_logger().info("Astar did not find a path....")
            return
        # self.get_logger().info(f"Astar Grid path: {self.grid_path}")

        self.publish_astar_path()

    def publish_astar_path(self):
        if self.costmap.data is None:
            return
        # Prepare the AstarPaths message
        for grid_point in self.grid_path:
            world_point = self.marker_to_publish.index_to_position_costmap(
                grid_point, self.costmap
            )
            self.world_path.append(world_point)

        astar_paths_msg = AstarPath()

        # Separate world path into x and y components
        astar_paths_msg.world_path_x = [point[0] for point in self.world_path]
        astar_paths_msg.world_path_y = [point[1] for point in self.world_path]

        # Separate grid path into x and y components
        astar_paths_msg.grid_path_x = [point[0] for point in self.grid_path]
        astar_paths_msg.grid_path_y = [point[1] for point in self.grid_path]

        self.publish_astar.publish(astar_paths_msg)
        self.grid_path = []
        self.world_path = []


def main(args=None):
    rclpy.init(args=args)
    astar_node = Astar()
    try:
        rclpy.spin(astar_node)
    finally:
        astar_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
