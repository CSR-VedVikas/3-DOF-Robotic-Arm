import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robotic_arm')
    urdf_path = os.path.join(pkg, 'urdf', 'arm.urdf')
    rviz_cfg  = os.path.join(pkg, 'config', 'arm.rviz')

    with open(urdf_path) as f:
        robot_desc = f.read()

    # ── shared obstacle parameters ─────────────────────────────────────────
    obs_params = {
        'obstacle_x':      1.2,
        'obstacle_y':      0.5,
        'obstacle_z':      0.3,
        'obstacle_radius': 0.25,
    }

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),

        # RViz2 auto-opens with the saved config
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='ultrasonic_node',
            parameters=[obs_params],
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='obstacle_node',
            parameters=[{**obs_params,
                         'stop_distance_cm': 30.0,
                         'warning_distance_cm': 60.0}],
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='planner_node',
            parameters=[{**obs_params, 'safety_margin': 0.35}],
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='waypoint_node',
            parameters=[{'interval_sec': 1.5}],
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='ik_node',
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='controller_node',
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='ee_marker_node',
            output='screen',
        ),

        Node(
            package='robotic_arm',
            executable='gui_node',
            output='screen',
        ),
    ])