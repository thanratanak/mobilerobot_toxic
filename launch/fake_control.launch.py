import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobilerobot_toxic')
    urdf_file = os.path.join(pkg_share, 'urdf', 'RobotNavigation.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'robot.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),
        Node(
            package='mobilerobot_toxic',
            executable='fake_mobile_base.py',
            name='fake_mobile_base',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])
