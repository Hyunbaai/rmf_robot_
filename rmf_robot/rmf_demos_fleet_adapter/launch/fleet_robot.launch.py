from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    initial_pose = ["-2.0", "-0.50", "0.0"]  # x, y, yaw 값 설정

    fleet_robot_node = Node(
        package="rmf_demos_fleet_adapter",  # 패키지 이름
        executable="fleet_robot",  # 실행 파일 이름
        output="screen",
        arguments=[
            "--robot_name", "turtlebot3",
            "--target_frame", "map",
            "--from_frame", "base_footprint",
            "--fleet_name", "turtlebot_fleet",
            "--initial_pose", *initial_pose,  # 초기 위치를 인자로 전달
        ],
    )

    return LaunchDescription([fleet_robot_node])
