from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_gps_waypoint_follower_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes/bases'), glob('meshes/bases/*')),
        (os.path.join('share', package_name, 'meshes/sensors'), glob('meshes/sensors/*')),
        (os.path.join('share', package_name, 'meshes/wheels'), glob('meshes/wheels/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'), 
         glob('models/turtlebot_waffle_gps/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps_1'), 
        glob('models/turtlebot_waffle_gps_1/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps_2'), 
        glob('models/turtlebot_waffle_gps_2/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'websocket_gps_waypoint_follower = nav2_gps_waypoint_follower_demo.websocket_gps_waypoint_follower:main',
            'websocket_gps_interactive_viewer = nav2_gps_waypoint_follower_demo.websocket_gps_interactive_viewer:main',
            'websocket_gps_point_controller = nav2_gps_waypoint_follower_demo.websocket_gps_point_controller:main',
            'websocket_gps_speed_controller = nav2_gps_waypoint_follower_demo.websocket_gps_speed_controller:main',
            'gps_waypoint_logger = nav2_gps_waypoint_follower_demo.gps_waypoint_logger:main',
            'gps_websocket_publisher = nav2_gps_waypoint_follower_demo.gps_websocket_publisher:main',
            'websocket_server = nav2_gps_waypoint_follower_demo.websocket_server:main',
        ],
    },
)
