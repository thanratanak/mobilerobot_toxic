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
    rviz_file = os.path.join(pkg_share, 'rviz', 'nav2.rviz')
    nav2_params_file = os.path.join(
        pkg_share, 'config', 'nav2_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        description='Full path to the saved map yaml file '
                    '(e.g. /home/nak/my_room_map.yaml)',
    )
    map_yaml_file = LaunchConfiguration('map')

    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Open teleop_twist_keyboard in its own xterm window',
    )
    enable_teleop = LaunchConfiguration('enable_teleop')

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        map_yaml_arg,
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

        # --- Localization: map_server + amcl ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params_file,
                {'yaml_filename': map_yaml_file},
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': localization_nodes,
            }],
        ),

        # --- Navigation: planning + control ---
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': navigation_nodes,
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
