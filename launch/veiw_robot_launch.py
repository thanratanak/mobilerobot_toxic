from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_urdf = PathJoinSubstitution(
        [FindPackageShare("mobilerobot_toxic"), "urdf", "RobotNavigation.urdf"]
    )
    rviz = PathJoinSubstitution(
        [FindPackageShare('mobilerobot_toxic'), 'rviz', 'description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf', 
            default_value=robot_urdf,
            description='URDF Robot'
        ),
        
        DeclareLaunchArgument(
            name='publish_joints', 
            default_value='true',
            description='joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='rviz'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    ])
