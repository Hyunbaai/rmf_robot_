import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

import xacro

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    gps_wpf_dir = get_package_share_directory(
        "nav2_gps_waypoint_follower_demo")
    gps_launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    # This checks that tb3 exists needed for the URDF. If not using TB3, its safe to remove.
    _ = get_package_share_directory('turtlebot3_gazebo')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='2.00'),
            'y': LaunchConfiguration('y_pose', default='-2.50'),
            'z': LaunchConfiguration('z_pose', default='0.30'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(gps_wpf_dir, 'config', 'nav2_no_map_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle_gps',
        description='name of the robot')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gps_wpf_dir, 'worlds', 'sonoma_raceway.world'),
        description='Full path to world model file to load')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(gps_wpf_dir, 'models', 'turtlebot_waffle_gps', 'model.sdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')
    
    configured_params = RewrittenYaml(
        source_file=params_file, root_key="", param_rewrites="", convert_types=True
    )

    models_dir = os.path.join(gps_wpf_dir, "models")
    models_dir += os.pathsep + \
        f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    set_gazebo_model_path_cmd = None

    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + \
            os.pathsep + models_dir
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gazebo_model_path)
    else:
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", models_dir)

    set_tb3_model_cmd = SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle")
    
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_launch_dir, 'double_dual_ekf_navsat.launch.py')
        ),
        launch_arguments={
            "namespace": namespace
        }.items(),
    )

    # robot_localization_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(gps_launch_dir, 'dual_ekf_navsat.launch.py')
    #     ),
    # )

    # print_yaml_file_cmd = ExecuteProcess(
    #     cmd=['cat', configured_params],
    #     output='screen'
    # )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": "True",
            "use_composition": "True",
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[gps_launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[gps_launch_dir], output='screen')

    # urdf = os.path.join(gps_wpf_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')
    # with open(urdf, 'r') as infp:
    #     robot_description = infp.read()

    urdf = os.path.join(gps_wpf_dir, "urdf", "turtlebot3_waffle_gps.urdf.xacro")
    # urdf = os.path.join(gps_wpf_dir, "urdf", "turtlebot3_waffle_gps_2.urdf.xacro")

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'frame_prefix': PythonExpression(["'", LaunchConfiguration('namespace'), "/'"]),
                     'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', urdf, ' robot_names:=', namespace, ' frame_prefix:=', namespace])
                    #  'robot_description': Command(['xacro ', urdf, ' robot_names:=', namespace])
                    }],
        arguments=[urdf],
        remappings=[('/tf', PythonExpression(["'/", LaunchConfiguration('namespace'), "/tf'"])),
                    ('/tf_static', PythonExpression(["'/", LaunchConfiguration('namespace'), "/tf_static'"]))]
        # remappings=remappings
    )

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', PathJoinSubstitution([namespace, 'waffle_gps']),
            '-file', robot_sdf,
            # '-topic', PathJoinSubstitution([namespace, 'robot_description']),
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    relay_topic = Node(
        package='topic_tools',
        executable = 'relay',
        name='relay',
        output='screen',
        parameters=[{
            'input_topic': PythonExpression(["'/", LaunchConfiguration('namespace'), "/tf'"]),
            'output_topic': "/tf", 
            'monitor_rate': 100.0}])

    relay_topics_tf_static = Node(
        package='topic_tools',
        executable = 'relay',
        name='relay',
        output='screen',
        parameters=[{
            'input_topic': PythonExpression(["'/", LaunchConfiguration('namespace'), "/tf_static'"]),
            'output_topic': "/tf_static",
            'monitor_rate': 100.0}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_tb3_model_cmd)

    ld.add_action(relay_topic)
    ld.add_action(relay_topics_tf_static)

    # ld.add_action(print_yaml_file_cmd)
    return ld