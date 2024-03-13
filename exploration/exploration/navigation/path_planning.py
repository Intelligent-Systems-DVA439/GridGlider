import heapq
import math

import numpy as np
from exploration.utility.helper_functions import euler_distance


def near_obstacle(lidar_data, threshold_distance=0.5):
    # get_logger("near_obstacle").info(f"lidar_data: {lidar_data}")
    distance = np.any(np.array(lidar_data) < threshold_distance)
    # get_logger("near_obstacle").info(f"distance: {distance}")
    return distance


def enhanced_pure_pursuit(
    current_x,
    current_y,
    current_heading,
    path,
    index,
    lookahead_distance,
    speed,
    max_steering_angle=math.pi / 4,
):
    """
    Improved pure pursuit logic.

    :param current_x: Current X position of the robot
    :param current_y: Current Y position of the robot
    :param current_heading: Current heading of the robot in radians
    :param path: The list of waypoints as (x, y) tuples
    :param index: The current index in the path that the robot is aiming for
    :param lookahead_distance: The distance ahead of the robot to target for the next waypoint
    :param speed: The desired speed of the robot
    :param max_steering_angle: The maximum steering angle to avoid sharp turns
    :return: Tuple of (velocity, steering angle, next index in the path)
    """
    closest_point = None
    v = speed

    # Find the first point in the path ahead of the lookahead distance
    for i in range(index, len(path)):
        x, y = path[i]
        distance = math.hypot(current_x - x, current_y - y)
        if distance > lookahead_distance:
            closest_point = (x, y)
            index = i
            break

    if closest_point is not None:
        # Calculate the angle to the closest point
        target_heading = math.atan2(
            closest_point[1] - current_y, closest_point[0] - current_x
        )
    else:
        # Default to the last waypoint if no suitable point is found
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        index = len(path) - 1

    # Calculate the desired steering angle
    desired_steering_angle = target_heading - current_heading
    # Normalize the steering angle to be within [-pi, pi]
    desired_steering_angle = (desired_steering_angle + math.pi) % (
        2 * math.pi
    ) - math.pi

    # Limit the steering angle to max_steering_angle and adjust velocity if necessary
    if abs(desired_steering_angle) > max_steering_angle:
        desired_steering_angle = max_steering_angle * math.copysign(
            1, desired_steering_angle
        )
        v = 0.0  # Stop the robot if the turn is too sharp

    return v, desired_steering_angle, index


def astar(costmap, start, goal):
    # Define the possible moves from a cell (8-directional movement)
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    # Initialize the closed set: places already evaluated
    close_set = set()
    # Initialize the came_from_map: to reconstruct the path later
    came_from = {}
    # Initialize the g_score map with the start position: cost from the start to current
    g_score = {start: 0}
    # Initialize the f_score map with the start position: total cost (g + heuristic)
    f_score = {start: euler_distance(start, goal)}
    # Open heap: positions to be evaluated, stored with their f_score for priority
    oheap = []

    # Add the start position to the open heap
    heapq.heappush(oheap, (f_score[start], start))

    # while we have positions to be evaluated
    while oheap:
        # Pop the position with the lowest f_score from the heap
        current = heapq.heappop(oheap)[1]

        # If the goal is reached, reconstruct and return the path
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]  # Return reversed path

        # Add the current position to the closed set
        close_set.add(current)

        # Iterate through all possible neighbor positions
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            # Calculate tentative_g_score for the neighbor
            tentative_g_score = g_score[current] + euler_distance(current, neighbor)

            # Check if the neighbor is within the costmap bounds
            if (
                0 <= neighbor[0] < costmap.shape[0]
                and 0 <= neighbor[1] < costmap.shape[1]
            ):
                # If the neighbor is an obstacle: skip it
                if costmap[neighbor[0]][neighbor[1]] == 100:  # Obstacle
                    continue
            else:
                # If the neighbor is out of bounds, skip it
                continue  # Out of bounds

            # If the neighbor is in the closed set and the new g_score is not better, skip it
            if neighbor in close_set and tentative_g_score >= g_score.get(
                neighbor, float("inf")
            ):
                continue

            # If its not in the open set, or we have a better g_score
            if tentative_g_score < g_score.get(
                neighbor, float("inf")
            ) or neighbor not in [i[1] for i in oheap]:
                # Update the path and scores
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + euler_distance(neighbor, goal)
                # Add the neighbor to the open set
                heapq.heappush(oheap, (f_score[neighbor], neighbor))

    # If the goal is not reachable, return False
    return False
