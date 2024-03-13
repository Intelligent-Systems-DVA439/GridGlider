import rclpy
from cluster_interfaces.msg import ClusterArray, ClusterInfo
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from .classes.structs import OccupancyGridInfo, OdomInfo, ParametersInfo
from .frontier.frontier import detect_frontier_points, select_target_cluster
from .utility.helper_functions import (costmap, create_occupancy_grid,
                                       euler_distance, euler_from_quaternion)


class Computation(Node):
    def __init__(self):
        super().__init__("computation")
        # Initalize structs
        self.parameters_info = ParametersInfo()
        self.logger = get_logger("computation Node")
        # Set default parameters
        self.declare_parameter("offset_to_the_boundary_points", 3)
        self.declare_parameter("occupied_threshold", 100)
        self.declare_parameter("cluster_size", 5)
        self.declare_parameter("eps", 2)
        self.declare_parameter("distance_to_cluster_threshold", 0.4)
        self.declare_parameter("computation_frequency", 1.0)
        self.declare_parameter("use_furthest_point", False)
        self.parameters_info.use_furthest_point = (
            self.get_parameter("use_furthest_point").get_parameter_value().bool_value
        )

        # Get parameters declared from the launch file
        self.parameters_info.computation_frequency = (
            self.get_parameter("computation_frequency")
            .get_parameter_value()
            .double_value
        )
        self.parameters_info.distance_to_cluster_threshold = (
            self.get_parameter("distance_to_cluster_threshold")
            .get_parameter_value()
            .double_value
        )
        self.parameters_info.offset_to_the_boundary_points = (
            self.get_parameter("offset_to_the_boundary_points")
            .get_parameter_value()
            .integer_value
        )

        self.parameters_info.cluster_size = (
            self.get_parameter("cluster_size").get_parameter_value().integer_value
        )
        self.parameters_info.eps = (
            self.get_parameter("eps").get_parameter_value().integer_value
        )
        self.parameters_info.occupied_threshold = (
            self.get_parameter("occupied_threshold").get_parameter_value().integer_value
        )
        self.get_logger().info(
            f"offset_to_the_boundary_points: {self.parameters_info.offset_to_the_boundary_points}"
        )
        self.get_logger().info(
            f"occupied_threshold: {self.parameters_info.occupied_threshold}"
        )
        self.get_logger().info(f"Cluster size: {self.parameters_info.cluster_size}")
        self.get_logger().info(f"eps: {self.parameters_info.eps}")

        self.computation_timer = self.create_timer(
            self.parameters_info.computation_frequency, self.compute_frontiers
        )
        # Subscription to the occupancy grid map
        self.occupancy_grid_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.occupancy_grid_callback, 10
        )

        self.robot_pose = self.create_subscription(
            Odometry, "/odom", self.robot_pose_callback, 10
        )

        # Used to message the draw node for visualization
        self.publish_frontiers = self.create_publisher(
            Float32MultiArray, "/frontiers", 10
        )

        self.publish_frontiers_cluster = self.create_publisher(
            ClusterArray, "/frontiers_cluster", 10
        )

        self.publish_costmap = self.create_publisher(OccupancyGrid, "/costmap", 2)  #

        self.publish_frontier_goal_world = self.create_publisher(
            Point, "/frontier_goal_world", 10
        )
        self.odom = OdomInfo()
        self.occupancy_grid = OccupancyGridInfo()
        self.old_nearest_cluster = None
        self.previous_clusters = []
        self.grid_update = False

    def robot_pose_callback(self, msg):
        """
        Get the quaternion position in free space
        """
        self.odom.current_velocity = msg.twist.twist.linear.x
        self.odom.data = msg
        self.odom.x = msg.pose.pose.position.x
        self.odom.y = msg.pose.pose.position.y
        self.odom.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def occupancy_grid_callback(self, msg):
        """
        Callback function whenever a new chunk of the map is retrieved
        """
        self.occupancy_grid.msg = msg
        self.occupancy_grid.resolution = msg.info.resolution
        self.occupancy_grid.width = msg.info.width
        self.occupancy_grid.height = msg.info.height
        self.occupancy_grid.originX = msg.info.origin.position.x
        self.occupancy_grid.originY = msg.info.origin.position.y
        self.occupancy_grid.data = msg.data
        self.compute_frontiers()

    def publish_clusters_data(self, clusters):
        # Create an instance of ClusterArray message
        cluster_array_msg = ClusterArray()

        # Iterate the clusters data to populate the message
        for cluster in clusters:
            label, centroid, size = cluster  # Unpack cluster data
            centroid_list = [
                float(value) for value in centroid.tolist()
            ]  # Ensure correct conversion

            # Create an instance of ClusterInfo for each cluster
            cluster_info_msg = ClusterInfo()
            cluster_info_msg.label = int(label)
            cluster_info_msg.centroid = centroid_list  # Convert numpy array to list
            cluster_info_msg.size = size

            # Append this ClusterInfo to the ClusterArray message
            cluster_array_msg.clusters.append(cluster_info_msg)

        # Publish the ClusterArray message
        self.publish_frontiers_cluster.publish(cluster_array_msg)

    def publish_frontiers_func(self, frontiers):

        frontiers_msg = Float32MultiArray()
        frontiers_msg.layout.dim.append(MultiArrayDimension())
        frontiers_msg.layout.dim[0].size = len(frontiers)
        frontiers_msg.layout.dim[0].stride = 2  # x and y
        frontiers_msg.layout.dim[0].label = "frontiers"
        flat_frontiers = [
            float(coord) for point in frontiers for coord in point
        ]  # Flatten the list of tuples
        frontiers_msg.data = flat_frontiers
        # self.logger.info(f"Publishing frontiers: {frontiers_msg.data}")
        self.publish_frontiers.publish(frontiers_msg)

    def is_close_enough(self, point_a, point_b, threshold=0.01):
        # Assuming euler_distance calculates the Euclidean distance between two points
        distance = euler_distance(point_a, point_b)
        return distance < threshold

    def compute_frontiers(self):
        if (
            self.occupancy_grid.data is None
            or self.occupancy_grid.width is None
            or self.occupancy_grid.height is None
        ):

            self.logger.info("Waiting for data...")
            return

        # Compute the costmap
        costmap_data = costmap(
            self.occupancy_grid.data,
            self.occupancy_grid.width,
            self.occupancy_grid.height,
            self.parameters_info.offset_to_the_boundary_points,
            self.parameters_info.occupied_threshold,
        )
        # Locate the frontiers
        frontiers = detect_frontier_points(
            self.occupancy_grid.height,
            self.occupancy_grid.width,
            self.parameters_info.occupied_threshold,
            costmap_data,
        )

        if not frontiers:
            self.logger.info(f"Frontiers: {frontiers}")
            return

        select_target_cluster_result = select_target_cluster(
            frontiers,
            self.odom,
            self.occupancy_grid,
            self.parameters_info.cluster_size,
            self.parameters_info.eps,
            self.parameters_info.use_furthest_point,
        )
        if select_target_cluster_result is None:
            self.logger.info(
                "No valid clusters found, or all clusters are behind the robot"
            )
            return

        # Extract data
        (
            labels_for_rviz,
            cluster_centroids,
            nearest_cluster,
            nearest_cluster_world,
            _,
        ) = select_target_cluster_result
        # self.logger.info(f"nearest_cluster_world type: {type(nearest_cluster_world)}")
        # Publish the centroids of the clusters
        self.publish_clusters_data(cluster_centroids)

        # Store in correct form
        if nearest_cluster_world is not None:
            goal_world_msg = Point(
                x=nearest_cluster_world[0], y=nearest_cluster_world[1], z=0.0
            )
            # Publishing frontier_goal_world
            self.publish_frontier_goal_world.publish(goal_world_msg)

        # Publishing frontiers
        self.publish_frontiers_func(frontiers)

        # Publish costmap
        self.publish_costmap.publish(
            create_occupancy_grid(
                costmap_data,
                self.occupancy_grid.resolution,
                self.occupancy_grid.originX,
                self.occupancy_grid.originY,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    computation_node = Computation()
    try:
        rclpy.spin(computation_node)
    finally:
        computation_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
