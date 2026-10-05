import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobilerobot_toxic')
    urdf_file = os.path.join(pkg_share, 'urdf', 'RobotNavigation.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    slam_params_file = os.path.join(
        pkg_share, 'config', 'slam_toolbox_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Open teleop_twist_keyboard in its own xterm window',
    )
    enable_teleop = LaunchConfiguration('enable_teleop')

    return LaunchDescription([
        enable_teleop_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='mobilerobot_toxic',
            executable='mecanum_fake_controller.py',
            name='mecanum_fake_controller',
            parameters=[{
                'wheel_radius': 0.0644,
                'wheelbase_x': 0.195,
                'wheelbase_y': 0.2729,
                'publish_rate': 50.0,
                'cmd_timeout': 0.5,
            }],
            output='screen',
        ),
        Node(
            package='mobilerobot_toxic',
            executable='fake_lidar_publisher.py',
            name='fake_lidar_publisher',
            parameters=[{
                'room_min_x': -5.0,
                'room_max_x': 5.0,
                'room_min_y': -5.0,
                'room_max_y': 5.0,
                'range_min': 0.12,
                'range_max': 8.0,
                'angle_increment_deg': 1.0,
                'publish_rate': 10.0,
                'noise_stddev': 0.0,
                'frame_id': 'laser_frame',
            }],
            output='screen',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params_file],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            prefix='xterm -T teleop_twist_keyboard -e',
            output='screen',
            condition=IfCondition(enable_teleop),
        ),
    ])