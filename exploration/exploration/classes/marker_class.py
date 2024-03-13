import numpy as np
from geometry_msgs.msg import Point
from rclpy.logging import get_logger
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class MarkerPublish:
    """
    Class which publishes a visual point in rviz
    """

    def __init__(self, parent_node):
        self.node = parent_node
        self.logger = get_logger("MarkerPublish Class")
        self.marker_publisher = self.node.create_publisher(
            Marker, "/visualization_marker", 10
        )

    def publish_astar_path(self, path, costmap):
        """
        Publishes the path calculated by the A* algorithm as a line strip in RViz.

        Args:
            path: The path as a list of tuples, where each tuple represents the grid coordinates (x, y).
            costmap: The costmap or grid object that includes resolution and origin properties.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "astar_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP for a continuous path
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0  # Neutral orientation

        # Set the scale of the marker. This determines the width of the line
        marker.scale.x = 0.05  # Line width

        # Set the color of the line strip
        marker.color = self.generate_color()

        # Convert each point in the path from grid coordinates to world coordinates
        # and add to the points list of the marker
        for grid_point in path:
            world_point = self.index_to_position_costmap(grid_point, costmap)
            marker.points.append(Point(x=world_point[0], y=world_point[1], z=0.0))

        # Publish the marker
        self.marker_publisher.publish(marker)

    def generate_color(self):
        color = ColorRGBA()  # Create a new ColorRGBA object
        color.r = (
            np.random.random()
        )  # Set red component to a random value between 0 and 1
        color.g = (
            np.random.random()
        )  # Set green component to a random value between 0 and 1
        color.b = (
            np.random.random()
        )  # Set blue component to a random value between 0 and 1
        color.a = 1.0  # Set alpha (opacity) to 1 (fully opaque)
        return color

    def index_to_position_costmap(self, index, occupancy_grid):
        """
        Convert the returned index to real coordinates
        """
        if index is None:
            self.logger.info(
                "Waiting for sensor, position data, robot pos, occupancy grid..."
            )
            return (0, 0)

        # Calculate (x, y) position in the map frame given an index in the occupancy grid
        grid_x, grid_y = index

        x = (grid_y * occupancy_grid.resolution) + occupancy_grid.originX
        y = (grid_x * occupancy_grid.resolution) + occupancy_grid.originY
        return (float(x), float(y))

    def index_to_position_frontiers(self, index, occupancy_grid):
        """
        Convert the returned index to real coordinates
        """
        if index is None:
            self.logger.info(
                "Waiting for sensor, position data, robot pos, occupancy grid..."
            )
            return (0, 0)

        # Calculate (x, y) position in the map frame given an index in the occupancy grid
        grid_x, grid_y = index

        x = (grid_x * occupancy_grid.resolution) + occupancy_grid.originX
        y = (grid_y * occupancy_grid.resolution) + occupancy_grid.originY
        return (float(x), float(y))

    def publish_costmap_data(self, costmap, grid):
        marker = Marker()
        marker.header.frame_id = "map"  # Assuming the frontiers are in the map frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "costmap_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0  # Neutral orientation

        marker.scale.x = 0.05  # Size of the points
        marker.scale.y = 0.05

        marker.color.r = 0.0  # Make the points blue
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Make the points completely opaque

        # Iterate through the costmap to find cells with specific values
        for row_index, row in enumerate(costmap):
            for col_index, cell in enumerate(row):
                if cell == 100:
                    # Convert cell indices to real-world coordinates
                    point = self.index_to_position_costmap((row_index, col_index), grid)
                    p = Point()
                    p.x = float(point[0])
                    p.y = float(point[1])
                    p.z = 0.0  # Assuming a 2D plane, z can be 0
                    marker.points.append(p)

        self.marker_publisher.publish(marker)

    def publish_frontier_markers(self, frontier_points, grid):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0  # Neutral orientation

        marker.scale.x = 0.05  # Size of the points
        marker.scale.y = 0.05

        marker.color.r = 0.0  # Make the points blue
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Make the points completely opaque

        # Convert frontier points to geometry_msgs/Point and add them to the marker
        frontier_coords = [
            self.index_to_position_frontiers(coord, grid) for coord in frontier_points
        ]
        # self.logger.info(f"frontier real world coordinates: {frontier_coords}")
        for point in frontier_coords:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.0  # Assuming a 2D plane, z can be 0
            marker.points.append(p)

        self.marker_publisher.publish(marker)

    def publish_target_marker(self, index, grid):
        if index is None:
            self.logger.info("Waiting for index occupancy grid...")
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = index.x
        marker.pose.position.y = index.y

        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0  # Neutral orientation
        marker.scale.x = 0.1  # Size of the marker [m]
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.marker_publisher.publish(marker)

    def publish_clustered_frontier_markers(self, frontier_points, cluster_labels, grid):
        unique_labels = set(cluster_labels)
        for label in unique_labels:
            if label == -1:
                continue  # Skip noise if using DBSCAN

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "frontiers_cluster_" + str(label)
            marker.id = int(label)
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1

            marker.color = self.generate_color()

            for point, point_label in zip(frontier_points, cluster_labels):
                if point_label == label:
                    world_point = self.index_to_position_frontiers(point, grid)
                    marker.points.append(
                        Point(x=float(world_point[0]), y=float(world_point[1]), z=0.0)
                    )

            self.marker_publisher.publish(marker)
