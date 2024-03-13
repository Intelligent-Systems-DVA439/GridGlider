# GridGlider

## Description

Utilizing fuzzy rules and A* for mapping and exploring unknown environments.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Technology Stack](#technology-stack)
- [Configuration](#configuration)
- [License](#license)

## Installation
Installation instructions for Ubuntu 22.04 MATE.

```bash
# Install required ROS 2 Packages
sudo apt update
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# Create your workspace
mkdir ~/maze_ws && cd ~/maze_ws
mkdir src && cd src

# Example command to clone your repository
git clone --depth 1 https://github.com/Intelligent-Systems-DVA439/GridGlider.git

# Install Gazebo_simulation package for Turtlebot3
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git

# Installation commands
pip install -r requirements.txt

# Build packages
cd ~/maze_ws/
colcon build --symlink-install
echo 'source ~/maze_ws/install/setup.bash'
echo 'export TURTLEBOT3_MODEL=waffle_pi'
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/bashrc
```

## Usage

```bash
# Example usage
ros2 launch maze_slam maze_online_async_launch.py
```

## Technology Stack

- Python 3.10
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo](https://classic.gazebosim.org/) for simulation

## Configuration
```bash
# Exploration config
~/maze_ws/src/exploration/config/params.yaml
```
```yaml
Explore:
  ros__parameters:
    speed: 0.26 # Float max 0.26
    max_steering_angle: 30.0 # Used to calculate the steering angle around corners
    lookahead_distance: 0.34 # Distance to lookahead in the path for pure pursuit
    prioritize_direction: False # not used
    near_obstacle: 0.27 # Used to detect walls and obstacles
    distance_to_waypoint_threshold: 0.3 # Distance to path waypoint to consider reached
    control_loop_period: 0.05 # How often to execute the navigation commands
    non_progress_threshold: 3 # How many tries until the path will reset
    progress_timeout_seconds: 5.0 # Seconds to trigger checking for navigation towards goal

Computation:
  ros__parameters:
    computation_frequency: 0.1 # Executions for the computations
    offset_to_the_boundary_points: 6 # Offset to inflate the walls keep below 6
    occupied_threshold: 100 # Cells to consider as occupied Integer
    cluster_size: 8 # Define minimum amount of samples to consider a cluster
    eps: 7 # Distance between clusters
    distance_to_cluster_threshold: 0.5 # Update the cluster navigation goal when the robot are this distance to the cluster
    use_furthest_point: True # Set False to select the furthest point in the nearest cluster, True for centroid

Draw:
  ros__parameters:
    publish_period: 0.5 # The period which to publish markers in RVIZ

```
```bash
# Slam parameters
~/maze_ws/src/maze_slam/config/mapper_params_online_async.yaml

```
After changing any configuration it could be useful to rebuild the project.

## License

```markdown
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
```
