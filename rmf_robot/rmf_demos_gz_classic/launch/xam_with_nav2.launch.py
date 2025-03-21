
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    print(f'bringup_dir: {bringup_dir}')
    # This checks that tb3 exists needed for the URDF. If not using TB3, its safe to remove.
    _ = get_package_share_directory('turtlebot3_gazebo')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
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

    declare_slam_cmd = DeclareLaunchArgument(
      'slam',
      default_value='False',
      description='Whether run a SLAM')

    # ========== user define ==========
    rmf_demos_maps_dir = get_package_share_directory('rmf_demos_maps')
    default_map_yaml = os.path.join(rmf_demos_maps_dir, 'xam', 'ingoffice_slam.yaml')
    print(f'default_map_yaml: {default_map_yaml}')
    declare_map_yaml_cmd = DeclareLaunchArgument(
      'map',
        default_value=default_map_yaml,
        description='Full path to map file to load')
    # ========== user define ==========

    declare_use_sim_time_cmd = DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
      description='Full path to the ROS2 parameters file to use for all launched nodes')
    print(f'params_file: {os.path.join(bringup_dir, "params", "nav2_params.yaml")}')

    declare_autostart_cmd = DeclareLaunchArgument(
      'autostart', default_value='true',
      description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
      'use_composition', default_value='True',
      description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
      'use_respawn', default_value='False',
      description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
      'rviz_config_file',
      default_value=os.path.join(
        bringup_dir, 'rviz', 'nav2_default_view.rviz'),
      description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
      'use_simulator',
      default_value='True',
      description='Whether to start the simulator')

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


    # ========== user define ==========
    default_office_world = os.path.join(rmf_demos_maps_dir, 'maps/xam', 'xam.world')
    print(f'default_office_world: {default_office_world}')
    declare_world_cmd = DeclareLaunchArgument(
      'world',
      default_value=default_office_world,
      description='Full path to world model file to load')
    # ========== user define ==========

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='name of the robot')

    default_robot_model = os.path.join(bringup_dir, 'worlds', 'waffle.model')
    print(f'default_robot_model: {default_robot_model}')
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=default_robot_model,
        description='Full path to robot sdf file to spawn the robot in gazebo')



    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())


    # ========== user define ==========
    gazebo_version = '11'  # Gazebo 버전 설정

    # GAZEBO_MODEL_PATH 환경 변수 설정을 위한 경로 설정
    world_path = os.path.join(
        get_package_share_directory('rmf_demos_maps'),
        'maps', 'xam', 'models'
    )
    print(f'world_path: {world_path}')

    # Gazebo 서버 실행 명령 설정
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],  # world_path 사용
        cwd=[launch_dir], output='screen')

    # Gazebo 클라이언트 실행 명령 설정
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient', '--verbose'],  # world_path 사용
        cwd=[launch_dir], output='screen')

    model_path_rmf_assets = os.path.join(  # not used
        get_package_share_directory('rmf_demos_assets'),
        'models'
    )
    gazebo_default_models = f"/usr/share/gazebo-{gazebo_version}/models"
    turtlebot3_models = "/opt/ros/iron/share/turtlebot3_gazebo/models"

    # GAZEBO_MODEL_PATH 환경 변수 설정
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f"{world_path}:{model_path_rmf_assets}:{gazebo_default_models}:{turtlebot3_models}"
    )

    final_gazebo_model_path = f"{world_path}/building_L1:{model_path_rmf_assets}:{gazebo_default_models}:{turtlebot3_models}"
    print(f"GAZEBO_MODEL_PATH will be set to: {final_gazebo_model_path}")

    # GAZEBO_RESOURCE_PATH 환경 변수 설정을 위한 경로 설정
    resource_path_rmf_assets = os.path.join(get_package_share_directory('rmf_demos_assets'))
    gazebo_default_resources = f"/usr/share/gazebo-{gazebo_version}"

    # GAZEBO_RESOURCE_PATH 환경 변수 설정
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=f"{resource_path_rmf_assets}:{gazebo_default_resources}"
    )

    final_gazebo_resource_path = f"{resource_path_rmf_assets}:{gazebo_default_resources}"
    print(f"GAZEBO_RESOURCE_PATH will be set to: {final_gazebo_resource_path}")


    # ========== user define ==========

    ld = LaunchDescription()
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_resource_path)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add any conditioned actions
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    # ld.add_action(start_gazebo_spawner_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld