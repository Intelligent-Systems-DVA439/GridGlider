import numpy as np
from sklearn.cluster import DBSCAN

from exploration.utility.helper_functions import (convert_grid_pos_to_world,
                                                  convert_world_to_grid_index)


def detect_frontier_points(
    height,
    width,
    occupied_threshold,
    costmap_data,
):
    frontier_points = []
    # Calculate search bounds within the occupancy grid
    # Define offsets to look around each cell (8-connected neighborhood)
    offsets = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    for y in range(height):
        for x in range(width):
            # Check if the current cell is unknown
            if costmap_data[y, x] == -1:
                for dy, dx in offsets:
                    ny, nx = y + dy, x + dx
                    # Check boundaries
                    if 0 <= ny < height and 0 <= nx < width:
                        # If one of the neighbors is known (explored), mark current as a frontier_point
                        if (
                            costmap_data[ny, nx] >= 0
                            and costmap_data[ny, nx] < occupied_threshold
                        ):
                            frontier_points.append((x, y))
                            break  # Dont check the other neighbors

    return frontier_points


def calculate_relative_angle(frontier_point, robot_position):
    # get_logger("calculate_relative_angle").info(
    #     f"frontier_point: {frontier_point}, robot_position: {robot_position}"
    # )
    dx = robot_position[0] - frontier_point[0]
    dy = robot_position[1] - frontier_point[1]
    angle_to_point = np.arctan2(dy, dx)

    return angle_to_point


def calculate_cluster_centroid(cluster_points):
    return np.mean(cluster_points, axis=0)


def select_target_cluster(
    frontier_points,
    robot_position,
    grid,
    cluster_size,
    eps,
    use_furthest_point_in_nearest_cluster=False,
):
    frontier_points_array = np.array(frontier_points)
    clustering = DBSCAN(eps=eps, min_samples=cluster_size).fit(frontier_points_array)
    labels = clustering.labels_
    unique_labels = set(labels) - {-1}

    if not unique_labels:
        return None

    robot_position_array = np.array(
        convert_world_to_grid_index(
            (robot_position.x, robot_position.y),
            (grid.originX, grid.originY),
            grid.resolution,
        )
    )

    nearest_cluster = None
    best_score = float("inf")
    clusters = []

    # First, find the nearest cluster based on the centroid or the closest point to the robot
    for label in unique_labels:
        cluster_mask = labels == label
        cluster_points = frontier_points_array[cluster_mask]

        # Always use centroid to determine the nearest cluster first
        centroid = calculate_cluster_centroid(cluster_points)
        angle_to_centroid = calculate_relative_angle(centroid, robot_position_array)
        alignment_score = min(
            abs(angle_to_centroid - robot_position.yaw),
            2 * np.pi - abs(angle_to_centroid - robot_position.yaw),
        )
        distance_to_robot = np.linalg.norm(centroid - robot_position_array)
        combined_score = distance_to_robot + alignment_score
        if combined_score < best_score:
            best_score = combined_score
            nearest_cluster = (label, centroid, cluster_points)

    if nearest_cluster is None:
        return None

    # Find the point within the selected cluster that is furthest from the robot
    if use_furthest_point_in_nearest_cluster:
        _, _, cluster_points = nearest_cluster
        distances = np.linalg.norm(cluster_points - robot_position_array, axis=1)
        furthest_point_index = np.argmax(distances)
        selected_target_point = cluster_points[furthest_point_index]
    else:
        _, selected_target_point, _ = nearest_cluster

    target_point_world = convert_grid_pos_to_world(selected_target_point, grid)

    return (
        labels,
        clusters,
        nearest_cluster[0],
        target_point_world,
        tuple(selected_target_point.astype(int)),
    )
