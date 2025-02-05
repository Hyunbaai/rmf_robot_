import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Directories
    gps_demo_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')

    # Launch configurations
    use_rviz = LaunchConfiguration('use_rviz', default='True')
    use_mapviz = LaunchConfiguration('use_mapviz', default='False')


    # Declare launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to launch RViz'
    )

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz', default_value='False', description='Whether to launch Mapviz'
    )

    # Include gps_waypoint_follower.launch.py
    gps_waypoint_follower_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_demo_dir, 'launch', 'gps_waypoint_follower.launch.py')
        ),
        launch_arguments={            
            'use_rviz': use_rviz,
            'use_mapviz': use_mapviz,
        }.items()
    )

    # Launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)

    # Add GPS Waypoint Follower
    ld.add_action(gps_waypoint_follower_cmd)

    return ld