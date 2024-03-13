from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.logging import get_logger
from sensor_msgs.msg import LaserScan

from ..utility.helper_functions import euler_from_quaternion
from .structs import OccupancyGridInfo, OdomInfo, SensorInfo


class SensorRead:
    """
    Sensor class which subscribes to the lidar topic, map topic and tf topic
    """

    def __init__(self, parent_node):
        # Subscription to tb3 lidar data
        self.node = parent_node
        self.logger = get_logger("SensorRead")

        # Lidar subscription
        self.lidar_subscription = self.node.create_subscription(
            LaserScan, "/scan", self.read_lidar, 10
        )
        # Subscription to the occupancy grid map
        self.occupancy_grid_subscription = self.node.create_subscription(
            OccupancyGrid, "/map", self.occupancy_grid_callback, 10
        )
        # Robot odometry frame, returned as quaternion
        self.robot_pose = self.node.create_subscription(
            Odometry, "/odom", self.robot_pose_callback, 10
        )

        self.lidar_data = None
        self.occupancy_grid = OccupancyGridInfo()
        self.odom = OdomInfo()
        self.sensor_info = SensorInfo()

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

    def read_lidar(self, msg):
        """
        Save the lidar sensor readings
        """
        self.lidar_data = msg.ranges
