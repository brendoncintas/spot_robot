from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'topic_sub_name':'/camera/hand/camera_info',
          'topic_pub_name':'/camera/hand/camera_info_repub'}
          ]

    return LaunchDescription([


        Node(
            package='spot_robot', executable='republish_camera_info', output='screen',
            parameters=parameters,
            arguments=[]),

    ])
