import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped
from rmf_fleet_msgs.msg import RobotState, Location, RobotMode, PathRequest
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
import argparse
from rmf_demos_fleet_adapter.utils.logger2 import info, warning, error, debug, critical

MODE_MAP = {
    "IDLE": RobotMode.MODE_IDLE,
    "CHARGING": RobotMode.MODE_CHARGING,
    "MOVING": RobotMode.MODE_MOVING,
    "PAUSED": RobotMode.MODE_PAUSED,
    "WAITING": RobotMode.MODE_WAITING,
    "EMERGENCY": RobotMode.MODE_EMERGENCY,
    "GOING_HOME": RobotMode.MODE_GOING_HOME,
    "DOCKING": RobotMode.MODE_DOCKING,
    "ADAPTER_ERROR": RobotMode.MODE_ADAPTER_ERROR,
    "CLEANING": RobotMode.MODE_CLEANING,
}


class FleetRobot(Node):
    def __init__(self, robot_name, target_frame, from_frame, fleet_name, initial_pose):
        super().__init__('fleet_robot_node')
        self.ongoing_request_cmd_id = None  # Current task ID.
        self.last_request_completed = None  # Last completed task ID.
        self.robot_name = robot_name  # Robot's name.
        self.target_frame = target_frame  # Reference frame (e.g., map).
        self.from_frame = from_frame  # Robot's frame (e.g., base_footprint).
        self.fleet_name = fleet_name  # Fleet name.

        # Nav2 Simple Commander
        self.navigator = BasicNavigator()
        debug('Creating BasicNavigator')

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state variables
        self.current_pose = None
        self.destination = None
        self.lock = threading.Lock()  # 쓰레드 안전성 보장
        self.battery_percent = 100.0  # Mock value, replace with actual battery logic
        self.path = []  # Current path for the robot
        self.sequence = 0  # Sequence number for RobotState

        # ROS2 publishers
        self.state_publisher = self.create_publisher(
            RobotState, f'/robot_state', QoSProfile(depth=10)
        )

        # ROS2 subscribers
        self.create_subscription(
            PathRequest,
            f'/robot_path_requests',
            self.path_request_cb,
            QoSProfile(depth=10),
        )

        # Initialize robot position
        self.set_initial_pose(initial_pose)

        # Start a thread for publishing robot state
        self.state_publishing_thread = threading.Thread(target=self.state_publisher_loop)
        self.state_publishing_thread.daemon = True  # Ensure thread exits with the main program
        self.state_publishing_thread.start()

    def state_publisher_loop(self):
        """Continuously publish robot state."""
        while rclpy.ok():  # Run while ROS is active
            self.publish_robot_state()
            time.sleep(1.0)  # Adjust the interval as needed

    def set_initial_pose(self, initial_pose):
        # Ensure Nav2 lifecycle is started
        # self.navigator.lifecycleStartup() 이미 활성화 상태

        # Set the initial pose
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = initial_pose["x"]
        pose.pose.position.y = initial_pose["y"]
        pose.pose.orientation.z = self.yaw_to_quaternion(initial_pose["yaw"])["z"]
        pose.pose.orientation.w = self.yaw_to_quaternion(initial_pose["yaw"])["w"]

        self.navigator.setInitialPose(pose)
        debug('Setting initial pose')

        # Wait until Nav2 is active
        self.navigator.waitUntilNav2Active()
        debug('Nav2 is active')

    # todo: stop 처리는 path[0], path[1]이 같은 경우로 일치시에 navigator.cancelTask() 호출
    def path_request_cb(self, msg):
        """
        Callback to handle PathRequest messages.
        """
        with self.lock:
            info(f"Received PathRequest with task_id: {msg.task_id}")

            # Check if the task is already being processed
            if self.ongoing_request_cmd_id and self.ongoing_request_cmd_id == msg.task_id:
                warning(
                    f"[{self.robot_name}] already processing task_id [{self.ongoing_request_cmd_id}] -- continuing as normal"
                )
                return

            # Process the path request
            if len(msg.path) < 2:
                warning("Invalid path received. At least 2 waypoints are required (current and destination).")
                return

            # Update the path
            self.path = msg.path  # Store the path for RobotState

            # Extract the destination pose (path[1])
            destination_pose = self.convert_waypoint_to_goal(msg.path[1])

            # Update the current task ID and start navigation
            self.ongoing_request_cmd_id = msg.task_id
            self.path = msg.path  # Store the path for RobotState
            self.navigate_to_goal(destination_pose)

    def convert_waypoint_to_goal(self, waypoint):
        """
        Convert a single waypoint to a PoseStamped goal for Nav2.
        Automatically calculates yaw based on direction from current position to waypoint.
        """
        # Get the current transform for the robot's position
        transform_stamped = self.get_current_transform()
        if not transform_stamped:
            warning(f"Unable to get the current transform for yaw calculation.")
            return None

        # Current robot position
        current_x = transform_stamped.transform.translation.x
        current_y = transform_stamped.transform.translation.y

        # Target position
        target_x = waypoint.x
        target_y = waypoint.y

        # Calculate yaw based on direction to target
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        calculated_yaw = math.atan2(delta_y, delta_x)

        # Create PoseStamped with calculated yaw
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.orientation.z = self.yaw_to_quaternion(calculated_yaw)["z"]
        pose.pose.orientation.w = self.yaw_to_quaternion(calculated_yaw)["w"]

        return pose

    def navigate_to_goal(self, goal_pose):
        """
        Send a single goal pose to Nav2 for execution using goToPose.
        """
        # Remove lifecycleStartup and lifecycleShutdown calls

        try:
            self.navigator.goToPose(goal_pose)

            # Monitor navigation progress
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and feedback.distance_remaining:
                    debug( f"Distance remaining: {feedback.distance_remaining:.2f} meters" )
                time.sleep(1.0)

            # Get the result of the navigation
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                info(f"Navigation succeeded for task_id: {self.ongoing_request_cmd_id or self.last_request_completed}")
                self.path = []  # Clear the path upon completion
            elif result == TaskResult.FAILED:
                warning(f"Navigation failed for task_id: {self.ongoing_request_cmd_id or self.last_request_completed}")
            else:
                warning(
                    f"Navigation canceled for task_id: {self.ongoing_request_cmd_id or self.last_request_completed}")

        except Exception as e:
            error(f"Error during navigation: {e}")

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return {"z": sy, "w": cy}

    def publish_robot_state(self):
        """
        Publish the robot's current state.
        """
        try:
            transform_stamped = self.get_current_transform()
            if not transform_stamped:
                return

            theta = math.atan2(
                2.0 * (transform_stamped.transform.rotation.w * transform_stamped.transform.rotation.z),
                1.0 - 2.0 * (transform_stamped.transform.rotation.z ** 2)
            )
            current_location = Location()
            current_location.x = transform_stamped.transform.translation.x
            current_location.y = transform_stamped.transform.translation.y
            current_location.yaw = theta
            current_location.t = self.get_clock().now().to_msg()
            current_location.level_name = "L1"  # Example: Static level name

            current_mode = RobotMode()
            current_mode.mode = self.determine_mode()

            # todo: self.navigator.isTaskComplete() 확인하여 current_task_id 처리
            if self.ongoing_request_cmd_id is not None and \
                    self.navigator.isTaskComplete():
                info(
                    f'Robot [{self.robot_name}] completed task with cmd id '
                    f'[{self.ongoing_request_cmd_id}]'
                )
                self.last_request_completed = self.ongoing_request_cmd_id
                self.ongoing_request_cmd_id = None

            robot_state_msg = RobotState()
            robot_state_msg.name = self.robot_name
            robot_state_msg.battery_percent = self.battery_percent
            robot_state_msg.location = current_location
            robot_state_msg.task_id = str(self.last_request_completed) if self.last_request_completed else ""
            robot_state_msg.mode = current_mode
            self.sequence += 1  # Increment sequence number
            robot_state_msg.seq = self.sequence
            robot_state_msg.path = [self.path[1]] if len(self.path) > 1 else []

            self.state_publisher.publish(robot_state_msg)

            info(f"RobotState details: name={robot_state_msg.name}, "
                 f"task_id={robot_state_msg.task_id}, "
                 f"path={robot_state_msg.path}, "
                 f"battery_percent={robot_state_msg.battery_percent}")

        except TransformException as e:
            warning(f"TransformException encountered: {e}")
        except AssertionError as e:
            error(f"AssertionError encountered: {e}")
        except Exception as e:
            error(f"Unexpected error in publish_robot_state: {e}")

    def determine_mode(self):
        """
        Determine the robot's current mode.
        """
        if not self.path or len(self.path) < 2:
            return MODE_MAP["IDLE"]
        return MODE_MAP["MOVING"]

    def get_current_transform(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.from_frame,
                rclpy.time.Time()
            )
            return transform_stamped
        except TransformException as ex:
            warning(f"Could not transform {self.from_frame} to {self.target_frame}: {ex}")
            return None

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Fleet Robot Node")
    parser.add_argument('--robot_name', type=str, required=True, help='Name of the robot')
    parser.add_argument('--target_frame', type=str, default='map', help='Target frame (e.g., map)')
    parser.add_argument('--from_frame', type=str, default='base_footprint', help='From frame (e.g., base_footprint)')
    parser.add_argument('--fleet_name', type=str, default='turtlebot', help='Fleet name (e.g., turtlebot)')
    parser.add_argument('--initial_pose', nargs=3, type=float, default=[-2.0, -0.50, 0.0], help='Initial pose [x, y, yaw]')
    # Parse known arguments only (ignore --ros-args)
    parsed_args, _ = parser.parse_known_args()

    # Prepare initial pose as dictionary
    initial_pose = {
        "x": parsed_args.initial_pose[0],
        "y": parsed_args.initial_pose[1],
        "yaw": parsed_args.initial_pose[2]
    }

    # Create and spin the FleetRobot node
    robot_node = FleetRobot(
        robot_name=parsed_args.robot_name,
        target_frame=parsed_args.target_frame,
        from_frame=parsed_args.from_frame,
        fleet_name=parsed_args.fleet_name,
        initial_pose=initial_pose
    )

    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
ros2 launch rmf_demos_fleet_adapter fleet_robot.launch.py

python fleet_robot_node.py --robot_name turtlebot3 \
                           --target_frame map \
                           --from_frame base_footprint \
                           --fleet_name turtlebot \
                           --initial_pose -2.0 -0.50 0.0
'''