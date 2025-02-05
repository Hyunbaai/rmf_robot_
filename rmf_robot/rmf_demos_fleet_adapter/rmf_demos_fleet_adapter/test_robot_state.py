import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_fleet_msgs.msg import RobotState
from rmf_demos_fleet_adapter.utils.logger2 import info, warning, error, debug, critical

class RobotStateSubscriber(Node):
    def __init__(self):
        super().__init__('robot_state_subscriber')

        # Define the QoS Profile
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL
        )

        # Create the subscription
        self.subscription = self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_cb,
            qos_profile=transient_qos
        )

        info('RobotState subscriber node has been started.')

    def robot_state_cb(self, msg):
        """Callback function to process received RobotState messages."""
        separator = "-" * 50
        info(separator)
        info('Received RobotState message:')
        info(f'  Name: {msg.name}')
        info(f'  Model: {msg.model}')
        info(f'  Task ID: {msg.task_id}')
        info(f'  Sequence: {msg.seq}')
        info(f'  Mode: {msg.mode.mode}')
        info(f'  Battery: {msg.battery_percent:.2f}%')
        info(f'  Location: (x: {msg.location.x}, y: {msg.location.y}, yaw: {msg.location.yaw})')
        info(f'  Path length: {len(msg.path)}')

        # Optionally print the path
        if msg.path:
            for i, loc in enumerate(msg.path):
                info(f'    Path[{i}]: (x: {loc.x}, y: {loc.y}, yaw: {loc.yaw})')
        info(separator)


def main(args=None):
    rclpy.init(args=args)

    robot_state_subscriber = RobotStateSubscriber()

    try:
        rclpy.spin(robot_state_subscriber)
    except KeyboardInterrupt:
        robot_state_subscriber.get_logger().info('Node stopped by user.')
    finally:
        robot_state_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
