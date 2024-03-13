#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        model_folder,
        "model.sdf",
    )
    x_pose = LaunchConfiguration("x_pose", default="-7")
    y_pose = LaunchConfiguration("y_pose", default="7.3")
    yaw_pose = LaunchConfiguration("yaw_pose", default="0.0")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    tb3_launch_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    param_file_dir = os.path.join(get_package_share_directory("maze_slam"), "config")

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    slam_params_file = os.path.join(
        param_file_dir,
        "mapper_params_online_async.yaml",
    )
    exploration_param_file = os.path.join(
        get_package_share_directory("exploration"), "config", "params.yaml"
    )
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            param_file_dir,
            "mapper_params_online_sync.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )
    declare_x_position_cmd = DeclareLaunchArgument(
        "x_pose", default_value=x_pose, description="Specify namespace of the robot"
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        "y_pose", default_value=y_pose, description="Specify namespace of the robot"
    )

    declare_yaw_position_cmd = DeclareLaunchArgument(
        "yaw_pose", default_value=yaw_pose, description="Specify namespace of the robot"
    )
    world = os.path.join(
        get_package_share_directory("maze_slam"),
        "worlds",
        "maze_1m_path_3_exits.world",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={"world": world, "output": "log"}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                "launch",
                "gzclient.launch.py",
            )
        ),
        launch_arguments={"output": "log"}.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_launch_dir,
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    spawn_robot_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            TURTLEBOT3_MODEL,
            "-file",
            urdf_path,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.01",
            "-Y",
            yaw_pose,
        ],
        output={"both": "log"},
    )
    start_sync_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
            {"log_level": "warn"},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output={"both": "log"},
    )
    explore_node = Node(
        package="exploration",
        executable="exploration_node",
        name="Explore",
        output="screen",
        parameters=[exploration_param_file],
    )
    computation_node = Node(
        package="exploration",
        executable="computation_node",
        name="Computation",
        output="screen",
        parameters=[exploration_param_file],
    )

    draw_node = Node(
        package="exploration",
        executable="draw_node",
        name="Draw",
        output="screen",
        parameters=[exploration_param_file],
    )
    astar_node = Node(
        package="exploration",
        executable="astar_node",
        name="Astar",
        output="screen",
        # parameters=[exploration_param_file],
    )

    rviz2_cmd = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("maze_slam"),
                "config",
                "rviz2_config.rviz",
            ),
        ],
        output={"both": "log"},
    )
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_yaw_position_cmd)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)

    ld.add_action(start_sync_slam_toolbox_node)

    ld.add_action(explore_node)
    ld.add_action(computation_node)
    ld.add_action(draw_node)
    ld.add_action(astar_node)

    ld.add_action(rviz2_cmd)

    return ld
