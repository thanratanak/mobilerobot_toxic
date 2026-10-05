"""Autonomous navigation in Gazebo with the saved map (Nav2 + AMCL).

    ros2 launch mobilerobot_toxic gazebo_nav.launch.py
    ros2 launch mobilerobot_toxic gazebo_nav.launch.py map:=/home/nak/my_map.yaml

In RViz: "2D Pose Estimate" if the robot is not where AMCL thinks, then
"Nav2 Goal" to send the robot somewhere.  Hold R1 on the PS5 pad to take over
manually (enable_joy:=true).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

PKG = 'mobilerobot_toxic'


def generate_launch_description():
    pkg = get_package_share_directory(PKG)
    lc = LaunchConfiguration

    def arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    # use_sim_time -> true and map path are rewritten into every node section
    params = RewrittenYaml(
        source_file=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        root_key='',
        param_rewrites={'use_sim_time': 'true', 'yaml_filename': lc('map')},
        convert_types=True)

    # Gazebo + robot + bridge (SLAM and its RViz are turned off).
    # NOTE: included launch arguments leak into this launch's configurations,
    # so the nav RViz uses its own argument name (use_rviz), not "rviz".
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo_slam.launch.py')),
        launch_arguments={
            'slam': 'false', 'rviz': 'false',
            'world': lc('world'), 'gui': lc('gui'),
            'enable_joy': lc('enable_joy'), 'enable_teleop': 'false',
            'x': lc('x'), 'y': lc('y'), 'yaw': lc('yaw'),
        }.items())

    def nav_node(package, executable, name, remaps=None):
        return Node(package=package, executable=executable, name=name, output='screen',
                    parameters=[params], remappings=remaps or [])

    # Controller output goes to cmd_vel_nav, the smoother republishes it on /cmd_vel
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    localization = [
        nav_node('nav2_map_server', 'map_server', 'map_server'),
        nav_node('nav2_amcl', 'amcl', 'amcl'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ]
    navigation = [
        nav_node('nav2_controller', 'controller_server', 'controller_server', to_nav),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server', to_nav),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                 to_nav + [('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['controller_server', 'planner_server',
                                         'behavior_server', 'bt_navigator',
                                         'velocity_smoother']}]),
    ]

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', os.path.join(pkg, 'rviz', 'nav2.rviz')],
                parameters=[{'use_sim_time': True}], condition=IfCondition(lc('use_rviz')))

    # start Nav2 after Gazebo and the robot are up (the robot spawns at ~10 s)
    nav2 = TimerAction(period=15.0, actions=localization + navigation + [rviz])

    return LaunchDescription([
        arg('map', os.path.expanduser('~/my_map.yaml'), 'Full path to the map yaml'),
        arg('world', 'custom_map.sdf', 'Gazebo world (must match the saved map)'),
        arg('gui', 'true', 'Show the Gazebo window'),
        arg('use_rviz', 'true', 'Start RViz with nav2.rviz'),
        arg('enable_joy', 'false', 'PS5 manual override (hold R1)'),
        arg('x', '0.0', 'Spawn x (also set amcl initial_pose in nav2_params.yaml)'),
        arg('y', '0.0', 'Spawn y'), arg('yaw', '0.0', 'Spawn yaw'),
        sim, nav2,
    ])