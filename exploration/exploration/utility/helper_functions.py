"""
Author: Pontus Svensson
Date: 2024-02
License: MIT License
"""

import math

import numpy as np
from nav_msgs.msg import OccupancyGrid

def calculate_direction_and_distance(odom, waypoint):
    # Calculate the angle (direction) and distance from the robot's current position to the waypoint
    robot_x, robot_y = odom.x, odom.y
    waypoint_x, waypoint_y = waypoint

    dx = waypoint_x - robot_x
    dy = waypoint_y - robot_y

    distance = math.sqrt(dx**2 + dy**2)
    angle_to_waypoint = math.atan2(dy, dx)

    yaw = odom.yaw
    direction = angle_to_waypoint - yaw
    direction = np.arctan2(math.sin(direction), math.cos(direction))
    # get_logger("calculate_direction_and_distance").info(
    #     f"Robot Yaw: {odom.yaw:.3f}, Angle to Waypoint: {angle_to_waypoint:.3f}, Calculated Direction: {direction:.3f}, Distance to waypoint: {distance}"
    # )
    return direction, distance


def create_occupancy_grid(costmap_data, resolution, originX, originY):
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info.resolution = resolution
    occupancy_grid.info.width = costmap_data.shape[1]
    occupancy_grid.info.height = costmap_data.shape[0]
    occupancy_grid.info.origin.position.x = originX  # Adjust as necessary
    occupancy_grid.info.origin.position.y = originY  # Adjust as necessary
    occupancy_grid.info.origin.position.z = 0.0  # Usually 0 for 2D grids
    occupancy_grid.info.origin.orientation.w = 1.0  # No rotation, facing upwards

    # Flatten the costmap_data and convert to the expected format
    # OccupancyGrid data is expected to be a list of int8 values
    occupancy_grid.data = costmap_data.flatten().astype(int).tolist()

    return occupancy_grid


def costmap(grid, width, height, lidar_range, occupied_threshold):
    if grid is None:
        return
    # get_logger("costmap").info(f"occupied_threshold value: {occupied_threshold}")
    # Reshape the data into a 2D numpy matrix
    costmap_data = np.array(grid).reshape(height, width)

    # Consider all cells greater than the occupied threshold as a wall
    wall_indices = np.where(costmap_data >= occupied_threshold)
    for offset_x in range(-lidar_range, lidar_range + 1):
        for offset_y in range(-lidar_range, lidar_range + 1):
            # Skip the cell the robot is located on
            if offset_x == 0 and offset_y == 0:
                continue

            # calculate the inflated coordinates
            inflated_x = np.clip(wall_indices[0] + offset_x, 0, height - 1)
            inflated_y = np.clip(wall_indices[1] + offset_y, 0, width - 1)

            # Set the cost for the inflated area
            costmap_data[inflated_x, inflated_y] = 100

    return costmap_data


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


def grid_path_to_world(path, origin_x, origin_y, resolution):
    world_path = []
    for grid_x, grid_y in path:
        # Convert grid coordinates to world coordinates
        world_x = origin_x + (grid_x * resolution)
        world_y = origin_y + (grid_y * resolution)
        # Append the converted coordinates to the world path
        world_path.append((world_x, world_y))
    return world_path


def convert_grid_pos_to_world(index, grid):
    grid_originX, grid_originY = grid.originX, grid.originY

    world_x = (index[0] * grid.resolution) + grid_originX
    world_y = (index[1] * grid.resolution) + grid_originY

    return world_x, world_y


def convert_grid_pos_to_world2(index, grid):
    grid_originX, grid_originY = grid.originX, grid.originY

    world_x = (index[1] * grid.resolution) + grid_originX
    world_y = (index[0] * grid.resolution) + grid_originY

    return world_x, world_y


def convert_robot_pos_to_grid(robot_position, grid_resolution):
    """Convert the robots real coordinates to the grid cells

    Args:
        robot_position (_type_): TFMessage.child_id_frame.{"base_link"}.transform.translation
        grid_resolution (_type_): OccupancyGrid

    Returns:
        robot_x_index, robot_y_index: The indexes corresponding to the robots positon on the occupancy grid
    """
    # Calculate indices of the robot's position in the grid
    robot_position_x, robot_position_y = robot_position.x, robot_position.y
    robot_x_index = int(robot_position_x / grid_resolution)
    robot_y_index = int(robot_position_y / grid_resolution)
    return robot_x_index, robot_y_index


def convert_world_to_grid_index(position, origin, resolution):
    """
    Convert a real-world position to grid indices.

    :param position: Tuple (x, y) representing the real-world position.
    :param origin: Tuple (x, y) representing the real-world coordinates of the grid's origin.
    :param resolution: The grid resolution (meters per cell).
    :return: Tuple (row_index, col_index) representing the indices in the grid.
    """
    dx = position[0] - origin[0]
    dy = position[1] - origin[1]
    col_index = int(dx / resolution)
    row_index = int(dy / resolution)
    return (row_index, col_index)


def euler_distance(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
