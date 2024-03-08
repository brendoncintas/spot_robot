from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'body',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'subscribe_scan_cloud': True,
          'approx_sync':True,
          'queue_size': 1000}]

    remappings=[
          ('rgb/image', '/camera/hand/image'),
          ('rgb/camera_info', '/camera/hand/camera_info'),
          ('depth/image', '/depth_registered/hand/image'),
          ('scan_cloud','/cloud_full'),
          ('odom', '/odometry')]

    return LaunchDescription([

        # Nodes to launch
      #  Node(
      #      package='rtabmap_odom', executable='rgbd_odometry', output='screen',
      #      parameters=parameters,
      #      remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

      #  Node(
      #      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
      #      parameters=parameters,
      #      remappings=remappings),
    ])
