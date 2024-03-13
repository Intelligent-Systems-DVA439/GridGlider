import numpy as np
import rclpy
from cluster_interfaces.msg import AstarPath, ClusterArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .classes.marker_class import MarkerPublish
from .classes.structs import ClusterClass, OccupancyGridInfo, ParametersInfo


class Draw(Node):
    def __init__(self):
        super().__init__("Draw")

        self.draw_markers = MarkerPublish(self)
        self.parameters_info = ParametersInfo()
        self.declare_parameter("publish_period", 1.0)
        self.parameters_info.publish_period = (
            self.get_parameter("publish_period").get_parameter_value().double_value
        )

        self.publish_timer = self.create_timer(
            self.parameters_info.publish_period, self.publish_all_markers
        )
        self.publish_astar_path = self.create_subscription(
            AstarPath, "/astar_path", self.astar_path_callback, 10
        )
        self.frontier_subscription = self.create_subscription(
            Float32MultiArray, "/frontiers", self.save_frontiers, 10
        )
        self.frontier_subscription = self.create_subscription(
            ClusterArray, "/frontiers_cluster", self.save_frontiers_cluster, 10
        )

        self.occupancy_grid_costmap = self.create_subscription(
            OccupancyGrid, "/costmap", self.unpack_costmap, 10
        )

        self.frontier_goal_world_subscription = self.create_subscription(
            Point, "/frontier_goal_world", self.frontier_goal, 10
        )
        self.world_path = None
        self.grid_path = None
        self.frontiers = None
        self.costmap = OccupancyGridInfo()
        self.clusters = ClusterClass()
        self.frontier_goal_world = None

    def publish_all_markers(self):
        if self.frontiers is not None:
            self.draw_markers.publish_frontier_markers(self.frontiers, self.costmap)
        if (
            hasattr(self.clusters, "cluster_centroids")
            and self.clusters.cluster_centroids
        ):
            self.draw_markers.publish_clustered_frontier_markers(
                self.clusters.cluster_centroids,
                self.clusters.cluster_labels,
                self.costmap,
            )
        if self.grid_path is not None:
            self.draw_markers.publish_astar_path(self.grid_path, self.costmap)
        if self.costmap.data is not None:
            self.draw_markers.publish_costmap_data(self.costmap.data, self.costmap)

    def astar_path_callback(self, msg):
        self.grid_path = []
        self.world_path = []
        self.world_path = list(zip(msg.world_path_x, msg.world_path_y))
        self.grid_path = list(zip(msg.grid_path_x, msg.grid_path_y))
        # self.get_logger().info(f"Astar callback: {self.grid_path}")
        # self.draw_markers.publish_astar_path(self.grid_path, self.costmap)

    def frontier_goal(self, msg):
        # Same as the cluster centroids and does not need to be plotted twice
        self.frontier_goal_world = msg

    def save_frontiers(self, msg):
        if self.costmap.data is None:
            return
        self.frontiers = None
        flat_list = list(msg.data)
        self.frontiers = [
            (int(flat_list[i]), int(flat_list[i + 1]))
            for i in range(0, len(flat_list), 2)
        ]
        # self.draw_markers.publish_frontier_markers(self.frontiers, self.costmap)

    def save_frontiers_cluster(self, msg):
        if self.costmap.data is None:
            return
        # Clear existing data to store new incoming data
        self.clusters.cluster_labels.clear()
        self.clusters.cluster_centroids.clear()
        self.clusters.cluster_sizes.clear()

        # Iterate through each ClusterInfo in the ClusterArray message
        for cluster_info in msg.clusters:
            # Append the data from each cluster to the class attributes
            self.clusters.cluster_labels.append(cluster_info.label)
            self.clusters.cluster_centroids.append(cluster_info.centroid)
            self.clusters.cluster_sizes.append(cluster_info.size)

        # self.draw_markers.publish_clustered_frontier_markers(
        #     self.clusters.cluster_centroids, self.clusters.cluster_labels, self.costmap
        # )

    def unpack_costmap(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.costmap.resolution = msg.info.resolution
        self.costmap.originX = msg.info.origin.position.x
        self.costmap.originY = msg.info.origin.position.y
        self.costmap.data = np.array(msg.data).reshape(height, width)
        # self.draw_markers.publish_costmap_data(self.costmap.data, self.costmap)


def main(args=None):
    """
    Main function which starts the programs and the nodes in a multithreaded environment
    """
    rclpy.init(args=args)
    draw_node = Draw()

    try:
        rclpy.spin(draw_node)
    finally:
        draw_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
