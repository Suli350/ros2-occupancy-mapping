"""Simulator + autonomous explorer (from ros2-obstacle-avoidance) + mapper + RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('obstacle_avoidance')
    share = get_package_share_directory('occupancy_mapping')
    world = os.path.join(sim_share, 'config', 'world.yaml')
    autonomous = LaunchConfiguration('autonomous')
    return LaunchDescription([
        DeclareLaunchArgument('autonomous', default_value='true',
                              description='false = drive with teleop_twist_keyboard'),
        Node(package='obstacle_avoidance', executable='world_sim', parameters=[world]),
        Node(package='obstacle_avoidance', executable='avoider', parameters=[world],
             condition=IfCondition(autonomous)),
        Node(package='occupancy_mapping', executable='mapper', output='screen',
             parameters=[os.path.join(share, 'config', 'mapper.yaml')]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'mapping.rviz')]),
    ])
