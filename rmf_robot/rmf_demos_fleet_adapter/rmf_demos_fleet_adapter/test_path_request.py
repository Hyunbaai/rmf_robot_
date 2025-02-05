
# ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# rmf
from rmf_fleet_msgs.msg import PathRequest, Location
from builtin_interfaces.msg import Time
from rmf_demos_fleet_adapter.utils.logger2 import info, warning, error, debug, critical

# fsm
import smach

# utils
import time
import math


def create_time(sec, nanosec):
    time_msg = Time()
    time_msg.sec = sec
    time_msg.nanosec = nanosec
    return time_msg


def create_location(x, y, yaw):
    location = Location()
    location.t = create_time(0, 0)  # 시간 초기화
    location.x = x
    location.y = y
    location.yaw = yaw
    location.obey_approach_speed_limit = False
    location.approach_speed_limit = 0.0
    location.level_name = 'L1'
    location.index = 0
    return location


class WaitForInputState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.result = None
    def execute(self, userdata):
        info("Press 'Enter' to repeat:")
        self.result = 'aborted' # execute is not allowed to static method
        while True:
            user_input = input().strip().lower()
            self.result = 'succeeded'
            break
        return self.result


class PathRequestPublisher(smach.State):
    def __init__(self, node, task_id, paths=None, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )

        self.timeout = timeout
        self.node = node

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.pub_ = self.node.create_publisher(PathRequest,
                                               '/robot_path_requests',
                                               qos_profile)
        if paths is None:
            raise ValueError("Paths parameter cannot be None")
        self.paths = paths

        self.current_task_id = task_id  # 초기 task_id
        self.current_path_index = 0  # 현재 경로 인덱스

    def execute(self, userdata):

        self.current_path_index = 0 # reset current_path_index

        while True:
            if self.current_path_index >= len(self.paths):
                info('All paths have been published. Stopping the node.')
                return 'succeeded'

            # 현재 경로 가져오기
            start, end = self.paths[self.current_path_index]

            # PathRequest 메시지 생성
            path_request = PathRequest()
            path_request.fleet_name = 'tinyRobot'
            path_request.robot_name = 'tinyRobot2'
            path_request.task_id = str(self.current_task_id)

            # 시작 위치
            location1 = create_location(start[0], start[1], 0.0)

            # 도착 위치
            location2 = create_location(end[0], end[1], 0.0)

            # 경로 추가
            path_request.path.append(location1)
            path_request.path.append(location2)

            self.pub_.publish(path_request)
            info(f'Published PathRequest with task_id: {path_request.task_id}, path: {start} to {end}')

            # 다음 경로로 이동
            self.current_path_index += 1
            self.current_task_id += 1

            debug(f'Waiting for {self.timeout} seconds')
            time.sleep(self.timeout)


def main(args=None):
    rclpy.init(args=args)
    node = Node('path_request_publisher')
    node.declare_parameter('test01', 100.0)
    node.declare_parameter('test02', False)
    smach.set_loggers(info, warning, debug, error)
    # Create state machine
    top = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'overall_success'])
    # top.userdata.blackboard = init_blackboard()

    paths0 = [
        [(8.15007, -9.809), (11.5510, -10.709)],  # 경로 1

    ]
    paths1 = [
        [(11.5510, -10.709), (8.15007, -9.809)],  # 경로 1
    ]
    with top:
        smach.StateMachine.add(
            'READY',
            WaitForInputState(),
            transitions={'succeeded': 'PATH0', 'aborted': 'aborted', 'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'PATH0',
            PathRequestPublisher(node, 1, paths0, 1),
            transitions={'succeeded': 'WAIT_FOR_INPUT', 'aborted': 'aborted', 'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'WAIT_FOR_INPUT',
            WaitForInputState(),
            transitions={'succeeded': 'PATH1', 'aborted': 'aborted', 'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'PATH1',
            PathRequestPublisher(node, 2, paths1, 1),
            transitions={'succeeded': 'READY', 'aborted': 'aborted', 'preempted': 'preempted'}
        )
    outcome = top.execute()
    if outcome == 'overall_success':
        info('State machine executed overall successfully')
    rclpy.shutdown()


if __name__ == '__main__':
    main()