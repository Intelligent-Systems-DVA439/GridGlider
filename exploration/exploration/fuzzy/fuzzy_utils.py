# fuzzy.py
# ==============================================================================
# Author: Carl Larsson, (modified by: Pontus Svensson)
# Description: Fuzzy functions
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
# ==============================================================================


# ------------------------------------------------------------------------------
# Libraries

# Base libraries
import math

import matplotlib.pyplot as plt
# Functional libraries
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from exploration.classes.structs import Range

# ------------------------------------------------------------------------------


# ==============================================================================
# Create fuzzy system
def create_fuzzy_system(
    defuzzy_method="centroid", visualize_memberships=0, resolution=0.0001
):
    # Fuzzy variables

    # Define fuzzy input variables (sensors)
    # Normalized
    left_sensor = ctrl.Antecedent(
        np.arange(
            0,
            1 + resolution,
            resolution,
        ),
        "left_sensor",
    )
    front_sensor = ctrl.Antecedent(
        np.arange(
            0,
            1 + resolution,
            resolution,
        ),
        "front_sensor",
    )
    right_sensor = ctrl.Antecedent(
        np.arange(
            0,
            1 + resolution,
            resolution,
        ),
        "right_sensor",
    )

    # Define fuzzy output variable (control signal)
    # Normalized
    linear_movement = ctrl.Consequent(
        np.arange(
            -1,
            1 + resolution,
            resolution,
        ),
        "linear",
    )
    angular_movement = ctrl.Consequent(
        np.arange(
            -1,
            1 + resolution,
            resolution,
        ),
        "angular",
    )

    # Define membership functions, using triangular and trapezoidal memberships
    # Normalized sensor readings memberships
    side_sensor_close_100 = 0.05  # Affects jittering towards wall
    side_sensor_close_0 = 0.15  # Affects jittering towards wall
    side_sensor_far_100 = 0.55  # Affects cornering and how close to corner it goes, aswell as willingness to enter openings

    left_sensor["close"] = fuzz.trapmf(
        left_sensor.universe, [-math.inf, 0, side_sensor_close_100, side_sensor_close_0]
    )  # "Lower", created using the fist value as being outside of range "to the left"
    left_sensor["medium"] = fuzz.trimf(
        left_sensor.universe,
        [side_sensor_close_100, side_sensor_close_0, side_sensor_far_100],
    )
    left_sensor["far"] = fuzz.trapmf(
        left_sensor.universe, [side_sensor_close_0, side_sensor_far_100, 1, math.inf]
    )  # "Upper", created using the last value outside of range "to the right"

    front_sensor["close"] = fuzz.trapmf(
        front_sensor.universe, [-math.inf, 0, 0.04, 0.08]
    )  # "Lower", created using the fist value as being outside of range "to the left"
    front_sensor["medium"] = fuzz.trimf(front_sensor.universe, [0.04, 0.08, 0.28])
    front_sensor["far"] = fuzz.trapmf(
        front_sensor.universe, [0.08, 0.28, 1, math.inf]
    )  # "Upper", created using the last value outside of range "to the right"

    right_sensor["close"] = fuzz.trapmf(
        right_sensor.universe,
        [-math.inf, 0, side_sensor_close_100, side_sensor_close_0],
    )  # "Lower", created using the fist value as being outside of range "to the left"
    right_sensor["medium"] = fuzz.trimf(
        right_sensor.universe,
        [side_sensor_close_100, side_sensor_close_0, side_sensor_far_100],
    )
    right_sensor["far"] = fuzz.trapmf(
        right_sensor.universe, [side_sensor_close_0, side_sensor_far_100, 1, math.inf]
    )  # "Upper", created using the last value outside of range "to the right"

    # Normalized control output memberships, use tirangular even at the edges since output has limits
    # Linear
    linear_movement["linear_reverse"] = fuzz.trimf(
        linear_movement.universe, [-1, -0.58, -0.15]
    )
    linear_movement["linear_stop"] = fuzz.trimf(
        linear_movement.universe, [-0.15, 0, 0.15]
    )
    linear_movement["linear_forward"] = fuzz.trimf(
        linear_movement.universe, [0.15, 0.58, 1]
    )

    # Angular
    # z-aix is positive counter clockwise, and negative clockwise (viewed from above)
    angular_movement["angular_right_fast"] = fuzz.trimf(
        angular_movement.universe, [-1, -0.73, -0.45]
    )
    angular_movement["angular_right_slow"] = fuzz.trimf(
        angular_movement.universe, [-0.55, -0.28, 0]
    )
    angular_movement["angular_stop"] = fuzz.trimf(
        angular_movement.universe, [-0.05, 0, 0.05]
    )
    angular_movement["angular_left_slow"] = fuzz.trimf(
        angular_movement.universe, [0, 0.28, 0.55]
    )
    angular_movement["angular_left_fast"] = fuzz.trimf(
        angular_movement.universe, [0.45, 0.73, 1]
    )

    # Visualize memberships
    if visualize_memberships:
        left_sensor.view()
        front_sensor.view()
        right_sensor.view()
        linear_movement.view()
        angular_movement.view()
        plt.show()

        # Linear movements based on front sensor distance
    rule1 = ctrl.Rule(
        front_sensor["close"], linear_movement["linear_reverse"]
    )  # Reverse if an obstacle is very close in front.
    rule2 = ctrl.Rule(
        front_sensor["medium"], linear_movement["linear_stop"]
    )  # Stop if an obstacle is at a medium distance in front.
    rule3 = ctrl.Rule(
        front_sensor["far"], linear_movement["linear_forward"]
    )  # Move forward if the path in front is clear.

    # Angular movements favoring left turn when possible
    rule4 = ctrl.Rule(
        left_sensor["far"] & front_sensor["far"] & right_sensor["far"],
        angular_movement["angular_left_slow"],
    )  # Slow left turn if all directions are clear, favoring exploration to the left. (changed to stop)
    rule5 = ctrl.Rule(
        left_sensor["far"] & front_sensor["far"] & right_sensor["medium"],
        angular_movement["angular_left_slow"],
    )  # Slow left turn if left and front are clear but right is medium.
    rule6 = ctrl.Rule(
        left_sensor["far"] & front_sensor["far"] & right_sensor["close"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if an obstacle is close on the right.

    rule7 = ctrl.Rule(
        left_sensor["far"] & front_sensor["medium"] & right_sensor["far"],
        angular_movement["angular_left_slow"],
    )  # Slow left turn if front is medium and left is clear.
    rule8 = ctrl.Rule(
        left_sensor["far"] & front_sensor["medium"] & right_sensor["medium"],
        angular_movement["angular_left_slow"],
    )  # Slow left turn if front and right are medium distance.
    rule9 = ctrl.Rule(
        left_sensor["far"] & front_sensor["medium"] & right_sensor["close"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if an obstacle is close on the right.

    rule10 = ctrl.Rule(
        left_sensor["far"] & front_sensor["close"] & right_sensor["far"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if an obstacle is very close in front.
    rule11 = ctrl.Rule(
        left_sensor["far"] & front_sensor["close"] & right_sensor["medium"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if front is close and right is medium.
    rule12 = ctrl.Rule(
        left_sensor["far"] & front_sensor["close"] & right_sensor["close"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if obstacles are close in front and right.

    # Angular movements favoring right turn or stop when left is blocked
    rule13 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["far"] & right_sensor["far"],
        angular_movement["angular_right_slow"],
    )  # Slow right turn if left is medium and front/right are clear.
    rule14 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["far"] & right_sensor["medium"],
        angular_movement["angular_stop"],
    )  # Stop if left is medium and right is medium.
    rule15 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["far"] & right_sensor["close"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if right is close.

    rule16 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["medium"] & right_sensor["far"],
        angular_movement["angular_right_slow"],
    )  # Slow right turn if left is medium.
    rule17 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["medium"] & right_sensor["medium"],
        angular_movement["angular_left_slow"],
    )  # Slow left turn if left and right are medium.
    rule18 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["medium"] & right_sensor["close"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if right is close.

    rule19 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["close"] & right_sensor["far"],
        angular_movement["angular_right_fast"],
    )  # Fast right turn if left is medium and front is close.
    rule20 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["close"] & right_sensor["medium"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if front is close and right is medium.
    rule21 = ctrl.Rule(
        left_sensor["medium"] & front_sensor["close"] & right_sensor["close"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if both front and right are close.

    # Angular movements for specific obstacle configurations
    rule22 = ctrl.Rule(
        left_sensor["close"] & front_sensor["far"] & right_sensor["far"],
        angular_movement["angular_right_fast"],
    )  # Fast right turn if an obstacle is close on the left.
    rule23 = ctrl.Rule(
        left_sensor["close"] & front_sensor["far"] & right_sensor["medium"],
        angular_movement["angular_right_fast"],
    )  # Fast right turn if left is close and right is medium.
    rule24 = ctrl.Rule(
        left_sensor["close"] & front_sensor["far"] & right_sensor["close"],
        angular_movement["angular_stop"],
    )  # Stop if obstacles are close on both sides.

    rule25 = ctrl.Rule(
        left_sensor["close"] & front_sensor["medium"] & right_sensor["far"],
        angular_movement["angular_right_fast"],
    )  # Fast right turn if left is close.
    rule26 = ctrl.Rule(
        left_sensor["close"] & front_sensor["medium"] & right_sensor["medium"],
        angular_movement["angular_right_fast"],
    )  # Fast right turn if left is close and front is medium.
    rule27 = ctrl.Rule(
        left_sensor["close"] & front_sensor["medium"] & right_sensor["close"],
        angular_movement["angular_stop"],
    )  # Stop if obstacles are close on all sides.

    rule28 = ctrl.Rule(
        left_sensor["close"] & front_sensor["close"] & right_sensor["far"],
        angular_movement["angular_right_fast"],
    )  # Fast right turn if left is close and front is close.
    rule29 = ctrl.Rule(
        left_sensor["close"] & front_sensor["close"] & right_sensor["medium"],
        angular_movement["angular_right_fast"],
    )  # Fast right turn if obstacles are close in front and on the left.
    rule30 = ctrl.Rule(
        left_sensor["close"] & front_sensor["close"] & right_sensor["close"],
        angular_movement["angular_left_fast"],
    )  # Fast left turn if obstacles are close on all sides, favoring left escape.

    # Create fuzzy control system
    fuzzy_ctrl = ctrl.ControlSystem(
        [
            rule1,
            rule2,
            rule3,
            rule4,
            rule5,
            rule6,
            rule7,
            rule8,
            rule9,
            rule10,
            rule11,
            rule12,
            rule13,
            rule14,
            rule15,
            rule16,
            rule17,
            rule18,
            rule19,
            rule20,
            rule21,
            rule22,
            rule23,
            rule24,
            rule25,
            rule26,
            rule27,
            rule28,
            rule29,
            rule30,
        ]
    )
    fuzzy_system = ctrl.ControlSystemSimulation(fuzzy_ctrl)

    # Define defuzzification method
    fuzzy_system.defuzzify_method = defuzzy_method

    return fuzzy_system


# ==============================================================================
# Fuzzy movement choice
def fuzzy_movement_choice(np_sensor_data, fuzzy_system):
    # Provide normalized sensor values to fuzzy system
    # For normalization and unnormalization
    sensor = Range(0, 3.51)
    linear = Range(-0.26, 0.26)
    angular = Range(-1.82, 1.82)

    left_sensor = (np.mean(np_sensor_data[10:80]) - sensor.min_val) / (
        sensor.max_val - sensor.min_val
    )
    front_sensor = (
        np.min(np.concatenate((np_sensor_data[-20:], np_sensor_data[0:20]), axis=None))
        - sensor.min_val
    ) / (sensor.max_val - sensor.min_val)

    right_sensor = (np.mean(np_sensor_data[-80:-10]) - sensor.min_val) / (
        sensor.max_val - sensor.min_val
    )

    # Lidar values go counter clockwise and start infront of the robot
    # Left value is mean value of a 70 degree cone to the left
    fuzzy_system.input["left_sensor"] = left_sensor
    # Front value is min value of a 40 degree cone forward
    fuzzy_system.input["front_sensor"] = front_sensor
    # Right value is mean value of a 70 degree cone to the right
    fuzzy_system.input["right_sensor"] = right_sensor
    # Fuzzy computation
    fuzzy_system.compute()

    # Fuzzy decision on which movement should be taken
    # Also "unnormalize" the data
    linear_value = (fuzzy_system.output["linear"] - (-1)) * (
        linear.max_val - linear.min_val
    ) / (1 - (-1)) + linear.min_val
    angular_value = (fuzzy_system.output["angular"] - (-1)) * (
        angular.max_val - angular.min_val
    ) / (1 - (-1)) + angular.min_val

    return linear_value, angular_value
