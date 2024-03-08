from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution([FindPackageShare('spot_robot'),
                                'params', 'main.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'spot_driver.launch.py'
                ])
            ]),
            launch_arguments={
                'config_file': params_file,
                'has_arm': 'True',
                'launch_rviz': 'False',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'point_cloud_xyzrgb.launch.py'
                ])
            ]),
            launch_arguments={
                'spot_name': '/',
                'camera': 'back',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'point_cloud_xyzrgb.launch.py'
                ])
            ]),
            launch_arguments={
                'spot_name': '/',
                'camera': 'frontleft',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'point_cloud_xyzrgb.launch.py'
                ])
            ]),
            launch_arguments={
                'spot_name': '/',
                'camera': 'frontright',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'point_cloud_xyzrgb.launch.py'
                ])
            ]),
            launch_arguments={
                'spot_name': '/',
                'camera': 'left',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'point_cloud_xyzrgb.launch.py'
                ])
            ]),
            launch_arguments={
                'spot_name': '/',
                'camera': 'right',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'point_cloud_xyzrgb.launch.py'
                ])
            ]),
            launch_arguments={
                'spot_name': '/',
                'camera': 'hand',
            }.items()
        ),

    ])

