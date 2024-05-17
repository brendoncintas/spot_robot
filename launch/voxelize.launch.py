from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'cloud_topic':'/local_grid',
          'frame_id':'body',
          'voxel_size':0.05,
          'published_topic_name':'local_grid_voxelized'}
          ]

    return LaunchDescription([
        Node(
            package='spot_robot', executable='voxelize', output='screen',
            parameters=parameters,
            arguments=[]),

    ])
