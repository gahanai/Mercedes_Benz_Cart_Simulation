from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        get_package_share_directory('my_robot_description'), 'urdf', 'Drwig.urdf'
    ])
    
    return LaunchDescription([
        # Declare the URDF path as a launch argument (optional)
        DeclareLaunchArgument(
            name='urdf_path',
            default_value=urdf_path,
            description='Full path to the URDF file to load'
        ),

        # Node for the robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')])
            }]
        ),

        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    get_package_share_directory('golf_cart_bringup'),
                    'world',
                    'empty_world.world'
                ])
            }.items()
        ),

        # Node to spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'Drwig'],
            output='screen'
        ),
    ])
