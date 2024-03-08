from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'body',
          'voxel_size':0.04,
          'msg_batch_size':10,
          'clip_distance':3.0}
          ]

    return LaunchDescription([


        Node(
            package='spot_robot', executable='publish_full_pointcloud', output='screen',
            parameters=parameters,
            arguments=[]),

    ])
