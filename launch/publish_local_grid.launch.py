from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[]

    return LaunchDescription([
        Node(
            package='spot_robot', executable='publish_local_grid.py', output='screen',
            parameters=parameters,
            arguments=[]),

    ])
