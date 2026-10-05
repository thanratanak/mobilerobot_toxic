import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('mobilerobot_toxic')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'RobotNavigation.urdf')
    
    # Verify URDF exists
    if not os.path.exists(urdf_file):
        raise RuntimeError(f"URDF file not found at {urdf_file}")
    
    # Read URDF contents
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Verify required mesh files exist
    required_meshes = [
        'base_link.STL',
        'wheel_fl_link.STL',
        'wheel_bl_link.STL',
        'wheel_fr_link.STL',
        'wheel_br_link.STL',
        'lidar_link.STL',
        'imu_link.STL'
    ]

    meshes_dir = os.path.join(pkg_share, 'meshes')
    for mesh in required_meshes:
        mesh_path = os.path.join(meshes_dir, mesh)
        if not os.path.exists(mesh_path):
            raise RuntimeError(f"Mesh file {mesh} not found in {meshes_dir}")
    
    # RViz config file (resolved at launch time)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('mobilerobot_toxic'),
        'rviz',
        'not_robot_model.rviz'
    ])

    # Declare launch argument to enable/disable RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with the robot model'
    )

    return LaunchDescription([
        rviz_arg,

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])
