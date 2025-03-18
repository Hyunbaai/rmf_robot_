#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import math
import yaml
import json
import time
import copy
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    DockSummary, RobotMode

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
from std_msgs.msg import String

import numpy as np
from pyproj import Transformer

import socketio

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

import threading

# fsm
import smach

# utils
from fsm_waypoint.utils import debug, info, warning, error, critical

# websocket
import websocket
import ssl
app = FastAPI()

def rmf_to_robot_transform(rmf_x, rmf_y, reference_coordinates):
    """
    Convert RMF coordinates to robot coordinates using affine transformation
    derived from reference points.

    Args:
        rmf_x (float): X coordinate in RMF frame.
        rmf_y (float): Y coordinate in RMF frame.
        reference_coordinates (dict): Reference coordinates with "rmf" and "robot" keys.

    Returns:
        (float, float): Transformed robot coordinates (x, y).
    """
    try:
        # Extract RMF and robot points from reference coordinates
        rmf_points = np.array(reference_coordinates["rmf"])
        robot_points = np.array(reference_coordinates["robot"])

        # Add homogeneous coordinates for affine transformation
        A = np.vstack([rmf_points.T, np.ones(len(rmf_points))]).T
        B = np.vstack([robot_points.T, np.ones(len(robot_points))]).T

        # Compute transformation matrix using least squares
        transformation_matrix, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

        # Apply transformation to RMF coordinates
        rmf_coords = np.array([rmf_x, rmf_y, 1])  # Homogeneous coordinates
        robot_coords = np.dot(transformation_matrix.T, rmf_coords)

        return robot_coords[0], robot_coords[1]
    except Exception as e:
        raise ValueError(f"Error in RMF to robot coordinate transformation: {e}")

class Request(BaseModel):
    map_name: Optional[str] = None
    activity: Optional[str] = None
    label: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_path_request = None
        self.last_completed_request = None
        self.mode_teleop = False
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = \
            self.svy_transformer.transform(gps_json['lat'], gps_json['lon'])
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_path_request is not None:
            if task_id != self.last_path_request.task_id:
                return False
        return True


# config_file:=/home/bcc/Works1/rmf_demos_robot/rmf_demos/config/turtlebot3_world/turtlebot3_world_config.yaml
class FleetManager(smach.State):
    def __init__(self, node, config, timeout=None):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.sub0_ = None
        self.sub1_ = None
        self.sub2_ = None
        self.debug = False
        self.config = config
        self.node = node
        self.timeout = timeout
        self.fleet_name = self.config['rmf_fleet']['name']
        self.robot_name = next(iter(self.config["rmf_fleet"]["robots"].keys()))
        mgr_config = self.config['fleet_manager']

        info(f"{self.fleet_name}:{self.robot_name}")

        self.url = 'wss://j1hwdvving.execute-api.ap-northeast-2.amazonaws.com/dev'
        self.auth = '66fc47ba2013b66a223b5c8a6fac0926'
        # self.connection_id = ''
        self.task_ids = []
        self.ws = None
        self.ws_thread = None
        self.connection_open = False  # WebSocket 연결 상태
        self.connection_closed = False  # 상태 전이를 위한 플래그
        self.sub_list = []  # 구독된 토픽들 목록

        self.fastapi_thread = threading.Thread(
            target=self.start_fastapi_server, daemon=True)

        info(f"Starting fleet manager at {mgr_config['ip']}:{mgr_config['port']}")
        self.fastapi_thread.start()

        self.gps = False
        self.offset = [0, 0]
        reference_coordinates_yaml = mgr_config.get('reference_coordinates')
        if reference_coordinates_yaml is not None:
            offset_yaml = reference_coordinates_yaml.get('offset')
            if offset_yaml is not None and len(offset_yaml) > 1:
                self.gps = True
                self.offset = offset_yaml

        # super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.action_paths = {}  # Map activities to paths

        for robot_name, _ in self.config['rmf_fleet']['robots'].items():  # server code로 인해서, 결국 한개임
            self.robots[robot_name] = State()
        assert(len(self.robots) > 0)

        profile = traits.Profile(geometry.make_final_convex_circle(
            self.config['rmf_fleet']['profile']['footprint']),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        self.vehicle_traits.differential.reversible =\
            self.config['rmf_fleet']['reversible']

        fleet_manager_config = self.config['fleet_manager']
        self.action_paths = fleet_manager_config.get('action_paths', {})
        self.sio = socketio.Client()
        self.timer = self.node.create_timer(0.2, self.timer_callback)

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.path_pub = self.node.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=transient_qos
        )

        @app.get('/open-rmf/rmf_demos_fm/status/',
                 response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }
            if robot_name is None:
                response['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    robot_state_data = self.get_robot_state(state, robot_name)  # 로봇 상태 데이터 생성
                    response['data']['all_robots'].append(robot_state_data)
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                response['data'] = self.get_robot_state(state, robot_name)
            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_demos_fm/navigate/',
                  response_model=Response)
        async def navigate(robot_name: str, cmd_id: int, dest: Request):
            info(f'navigate: {robot_name}, cmd_id: {cmd_id}, '
                 f'x: {dest.destination["x"]:.3f}, '
                 f'y: {dest.destination["y"]:.3f}, '
                 f'yaw: {dest.destination["yaw"]:.3f}'
                 )
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or len(dest.destination) < 1):
                return response

            # RMFGoTOPose에서 변경
            # target_x_rmf = dest.destination['x']
            # target_y_rmf = dest.destination['y']
            # target_x, target_y = rmf_to_robot_transform(target_x_rmf, target_y_rmf, self.reference_coordinates)

            target_x = dest.destination['x']
            target_y = dest.destination['y']
            target_yaw = dest.destination['yaw']
            target_map = dest.map_name
            target_speed_limit = dest.speed_limit

            # target_x -= self.offset[0]
            # target_y -= self.offset[1]

            debug(f'Debug for target_x, target_y: {target_x},{target_y}')

            t = self.node.get_clock().now().to_msg()

            path_request = PathRequest()
            robot = self.robots[robot_name]
            if robot.state is None:
                return response

            cur_x = robot.state.location.x
            cur_y = robot.state.location.y
            cur_yaw = robot.state.location.yaw
            cur_loc = robot.state.location
            path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(disp/self.vehicle_traits.linear.nominal_velocity) +\
                int(abs(abs(cur_yaw) - abs(target_yaw)) /
                    self.vehicle_traits.rotational.nominal_velocity)
            t.sec = t.sec + duration
            target_loc = Location()
            target_loc.t = t
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.yaw = target_yaw
            target_loc.level_name = target_map
            target_loc.obey_approach_speed_limit = False
            if target_speed_limit is not None and target_speed_limit > 0.0:
                target_loc.obey_approach_speed_limit = True
                target_loc.approach_speed_limit = target_speed_limit

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            self.path_pub.publish(path_request)

            if self.debug:
                print(f'Sending navigate request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = target_loc

            response['success'] = True
            return response

        @app.get('/open-rmf/rmf_demos_fm/stop_robot/',
                 response_model=Response)
        async def stop(robot_name: str, cmd_id: int):
            warning(f"stop_robot: {robot_name}, {cmd_id}")
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            if robot.state is None:
                return response

            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            # Appending the current location twice will effectively tell the
            # robot to stop
            path_request.path.append(robot.state.location)
            path_request.path.append(robot.state.location)

            path_request.task_id = str(cmd_id)
            self.path_pub.publish(path_request)

            info(f'Sending stop request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = None

            response['success'] = True
            return response

        @app.get('/open-rmf/rmf_demos_fm/action_paths/',
                 response_model=Response)
        async def action_paths(activity: str, label: str):
            response = {'success': False, 'msg': ''}
            if activity not in self.action_paths:
                return response

            if label not in self.action_paths[activity][label]:
                return response

            response['data'] = self.action_paths[activity][label]
            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_demos_fm/start_activity/',
                  response_model=Response)
        async def start_activity(
            robot_name: str,
            cmd_id: int,
            request: Request
        ):
            warning(f"start_activity: {robot_name}, {cmd_id}, {request}")
            response = {'success': False, 'msg': ''}
            if (
                robot_name not in self.robots
                or request.activity not in self.action_paths
                or request.label not in self.action_paths[request.activity]
            ):
                return response

            robot = self.robots[robot_name]

            if robot.state is None:
                return response

            path_request = PathRequest()
            cur_loc = robot.state.location
            target_loc = Location()
            path_request.path.append(cur_loc)

            activity_path = self.action_paths[request.activity][request.label]
            map_name = activity_path['map_name']
            for wp in activity_path['path']:
                target_loc = Location()
                target_loc.x = wp[0]
                target_loc.y = wp[1]
                target_loc.yaw = wp[2]
                target_loc.level_name = map_name
                path_request.path.append(target_loc)

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(cmd_id)
            self.path_pub.publish(path_request)

            if self.debug:
                print(
                    f'Sending [{request.activity}] at [{request.label}] '
                    f'request for {robot_name}: {cmd_id}'
                )
            robot.last_path_request = path_request
            robot.destination = target_loc

            response['success'] = True
            response['data'] = {}
            response['data']['path'] = activity_path
            return response

        @app.post('/open-rmf/rmf_demos_fm/toggle_teleop/',
                  response_model=Response)
        async def toggle_teleop(robot_name: str, mode: Request):
            warning(f"toggle_teleop: {robot_name}, {mode}")

            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots):
                return response
            # Toggle action mode
            self.robots[robot_name].mode_teleop = mode.toggle
            response['success'] = True
            return response

    def timer_callback(self):
        if self.connection_open:
            robot_name = next(iter(self.robots))
            state = self.robots.get(robot_name)
            if state is None or state.state is None:
                return
            
            formatted_data = self.format_robot_state(self.get_robot_state(state, robot_name))
            # debug(f'timer_callback: {formatted_data}')
            try:
                self.ws.send(formatted_data)  # WebSocket으로 데이터 전송
            except Exception as e:
                print(f"Failed to send data: {e}")

    def robot_state_cb(self, msg):
        if (msg.name in self.robots):
            robot = self.robots[msg.name]
            # info(
            #     f"robot_state_cb: {msg.name}, {msg.task_id}, {msg.mode.mode}, {msg.location.x}, {msg.location.y}"
            # )

            # robot.is_expected_task_id(msg.task_id) true일 가능성이 매우 높음, robot.mode_teleop false일 가능성이 매우 높음 => robot 상태를 업데이트함
            # robot.is_expected_task_id(msg.task_id) false면 이전 task 가 수행중인것이고, teleop가 false이면 => 최신 task path로 보내고
            # robot.is_expected_task_id(msg.task_id) false 이고 teleop가 동작중이면 true이면 => robot 상태를 업데이트함
            # robot.is_expected_task_id(msg.task_id) true이고 즉 정상적으로 처리되고, teleop가 동작중이면 => robot 상태를 업데이트함
            if not robot.is_expected_task_id(msg.task_id) and not robot.mode_teleop:
                # This message is out of date, so disregard it.
                if robot.last_path_request is not None:
                    # Resend the latest task request for this robot, in case
                    # the message was dropped.
                    # warning(
                    #     f'Republishing task request for {msg.name}: '
                    #     f'{robot.last_path_request.task_id}, '
                    #     f'because it is currently following {msg.task_id}'
                    # )
                    self.path_pub.publish(robot.last_path_request)
                    # 21, 22 처럼 버전이 일치 하지 않으면 즉 task_id가 이전 것이면 publish 하는데 KEEP_LAST와 depth=1로 90hz로 들어오는
                    # robot_state에서 연속적으로 전송하지 않는 역할을 함.
                    # 그럼 이전 버전이 왜 오는가? waypoint가 완료되기 직전 90hz로 인해서 데이터가 연속해서 들어옴
                    # todo: 이전 버전이 들어오면 publish 하지 않고 return ?
                    # fsm에서 path_request_cb() 내에서는 진행중인 task_id와 일치만 하지 않아도 통과 되므로
                    # 이전 버전도 goToPose로 진행하는건 문제가 있음.
                    # debug(f"robot_state_cb [REPUB]: "
                    #       f"{msg.name}, "
                    #       f"Complete or Ongoing task_id: {msg.task_id}, "
                    #       f"Last received task_id {robot.last_path_request.task_id}")
                return

            robot.state = msg
            # warning(f'robot_state_cb [UPDATE]: {robot.state.location.x}, {robot.state.location.y}')
            # Check if robot has reached destination
            if robot.destination is None:
                return

            if (msg.mode.mode == RobotMode.MODE_IDLE or msg.mode.mode == RobotMode.MODE_CHARGING) and len(msg.path) == 0:
                robot = self.robots[msg.name]
                robot.destination = None
                completed_request = int(msg.task_id)
                if robot.last_completed_request != completed_request:
                    debug(f'Detecting completed request for {msg.name}: ' 
                          f'{completed_request}' )
                robot.last_completed_request = completed_request

                # formatted_data = self.format_robot_state(self.get_robot_state(robot, msg.name))
                # debug(f"[REPUB]: {formatted_data}")

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if (fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, robot: State, robot_name):
        data = {}
        if self.gps:
            # position = copy.deepcopy(robot.gps_pos)
            position = [robot.state.location.x, robot.state.location.y]
        else:
            position = [robot.state.location.x, robot.state.location.y]
            # info(f"Location: {robot.state.location.x}, {robot.state.location.y}")
        angle = robot.state.location.yaw
        data['fleet_name'] = self.fleet_name
        data['robot_name'] = robot_name
        data['map_name'] = robot.state.location.level_name
        data['position'] =\
            {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = robot.state.battery_percent
        if (robot.destination is not None
                and robot.last_path_request is not None):
            destination = robot.destination
            # remove offset for calculation if using gps coords
            if self.gps:
                position[0] -= self.offset[0]
                position[1] -= self.offset[1]
            # calculate arrival estimate
            dist_to_target =\
                self.disp(position, [destination.x, destination.y])
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (dist_to_target /
                        self.vehicle_traits.linear.nominal_velocity +
                        ori_delta /
                        self.vehicle_traits.rotational.nominal_velocity)
            cmd_id = int(robot.last_path_request.task_id)
            data['destination_arrival'] = {
                'cmd_id': cmd_id,
                'duration': duration
            }
        else:
            data['destination_arrival'] = None

        data['last_completed_request'] = robot.last_completed_request
        if (
            robot.state.mode.mode == RobotMode.MODE_WAITING
            or robot.state.mode.mode == RobotMode.MODE_ADAPTER_ERROR
        ):
            info(f"Replan for {robot_name}, mode: {robot.state.mode.mode}")
            # The name of MODE_WAITING is not very intuitive, but the slotcar
            # plugin uses it to indicate when another robot is blocking its
            # path.
            #
            # MODE_ADAPTER_ERROR means the robot received a plan that
            # didn't make sense, i.e. the plan expected the robot was starting
            # very far from its real present location. When that happens we
            # should replan, so we'll set replan to true in that case as well.
            data['replan'] = True
        else:
            data['replan'] = False

        return data

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

    def execute(self, userdata):
        self.sub_list = []  # reset
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.sub0_ = self.node.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_cb,
            qos_profile=transient_qos
        )
        self.sub1_ = self.node.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)
        '''
        ros2 topic pub -1 --keep-alive 5 /rmf_patrol_request std_msgs/msg/String "data: 'robot_task_request,patrol,t1 t2 t3'" --qos-durability transient_local
        ros2 topic pub -1 --keep-alive 5 /rmf_patrol_request std_msgs/msg/String "data: 'cancel_task_request,0,0'" --qos-durability transient_local
        
        '''
        self.sub2_ = self.node.create_subscription(
            String,
            '/rmf_patrol_request',
            self.rmf_patrol_request_callback,
            qos_profile=transient_qos)

        self.sub_list.append(self.sub0_)
        self.sub_list.append(self.sub1_)
        self.sub_list.append(self.sub2_)

        start_time = time.time()
        info("FleetManager state executing...")
        self.connection_open = False  # reset
        self.connection_closed = False  # reset
        if self.robots:
            # 첫 번째 키를 robot_name에 저장
            robot_name = next(iter(self.robots))
            print(f"First robot_name: {robot_name}")
        else:
            raise ValueError("self.robots is empty, no robot_name available")

        # 헤더 생성
        headers = {
            "Auth": self.auth,  # 인증 토큰
            "fleet_name": self.fleet_name,
            "robot_name": robot_name  # robot_name 추가
        }
        self.connect(headers)

        outcome = 'succeeded' # because timeout not mandatory

        try:
            while rclpy.ok():
                if self.preempt_requested():
                    self.service_preempt()
                    self.cleanup()  # 구독 해제 및 WebSocket 종료
                    outcome = 'preempted'
                    break

                if self.connection_closed:
                    self.cleanup_only_wss()  # 구독 해제 및 WebSocket 종료, ws은 다시 연결하는데 sub은?
                    # outcome = 'aborted'
                    # break
                    self.connection_open = False  # reset
                    self.connection_closed = False  # reset
                    self.connect(headers)  # 다시 연결 시도
                    time.sleep(1)

                time.sleep(0.1)  # Adjust the interval as needed

                if self.timeout:
                    if time.time() - start_time > self.timeout:
                        outcome = 'succeeded'
                        break
        except KeyboardInterrupt:
            error("Interrupted by user, shutting down...")
            self.cleanup()
            return 'preempted'
        except Exception as e:
            error(f"An error occurred while creating waypoint: {str(e)}")
            self.cleanup()
            return 'aborted'

        self.cleanup()
        return outcome

    def connect(self, headers):
        self.ws = websocket.WebSocketApp(self.url,
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close,
                                         header=headers)
        self.ws_thread = threading.Thread(target=self.ws.run_forever,
                                          kwargs={"sslopt": {"cert_reqs": ssl.CERT_NONE}})
        self.ws_thread.start()

    def on_open(self, ws):
        self.connection_open = True
        info("Connection opened")

    def on_message(self, ws, message):
        # info('Received message: ' + message)
        try:
            package = json.loads(message)
            if package.get('type') == 'connected':
                args = package['args']
                connection_id = args.get('connection_id')
                info(f'connection_id: {connection_id}')
            elif package.get('type') == 'navigate':
                data = package.get('data')
                robot_name = data.get('robot_name')
                cmd_id = data.get('cmd_id')
                dest = data.get('destination')
                map_name = data.get('map_name')
                speed_limit = data.get('speed_limit')
                response = self.navigate(robot_name, cmd_id, dest, map_name, speed_limit)
                # info(f'response: {response}')
            elif package.get('type') == 'stop_robot':
                data = package.get('data')
                robot_name = data.get('robot_name')
                cmd_id = data.get('cmd_id')
                response = self.stop_robot(robot_name, cmd_id)
                # info(f'response: {response}')
            elif package.get('type') == 'response':
                args = package.get('args')
                data = args.get('message')
                info(f'Get Response message: {data}')

                task_type = data.get("type", "default_robot_task_request")

                # robot_task_request이면 task_id 저장
                if task_type == "robot_task_request":
                    task_id = data.get("task_id")
                    if task_id:
                        self.task_ids.append(task_id)
                        info(f'Stored task_id: {task_id} for robot {data["robot_name"]}')
                    else:
                        info(f'No task_id found in response: {data}')

                elif task_type == "cancel_task_request":
                    info(f'Received cancel_task_request response for task_id: {data.get("task_id")}')
            else:
                error(f"Unknown message type.{package}")

        except json.JSONDecodeError:
            error("The received message is not in JSON format.")

    def on_error(self, ws, error):
        self.connection_open = False
        error(f"WebSocket error: {error}")
        self.connection_closed = True  # 상태 전이를 위한 플래그 설정

    def on_close(self, ws, close_status_code, close_msg):
        self.connection_open = False
        info(f"WebSocket connection closed, code: {close_status_code}, msg: {close_msg}")
        self.connection_closed = True  # 상태 전이를 위한 플래그 설정

    def cleanup(self):
        """구독 해제 및 WebSocket 종료"""
        info("Cleaning up sub_list and closing WebSocket...")
        for sub in self.sub_list:
            self.node.destroy_subscription(sub)  # 모든 구독 해제

        if self.ws:
            self.ws.close()
            if self.ws_thread and self.ws_thread.is_alive():
                self.ws_thread.join()

    def cleanup_only_wss(self):
        if self.ws:
            self.ws.close()
            if self.ws_thread and self.ws_thread.is_alive():
                self.ws_thread.join()

    def format_robot_state(self, robot_state_data):
        # 기본 포맷에 맞게 데이터 구성
        temp = [
            {
                'data': robot_state_data
            }  # 로봇 상태 데이터 포함
        ]
        # JSON 직렬화하여 반환
        data_to_send = json.dumps(temp)
        return data_to_send

    def start_fastapi_server(self):
        uvicorn.run(app,
                    host=self.config['fleet_manager']['ip'],
                    port=self.config['fleet_manager']['port'],
                    log_level="warning"
                    )

    def send_task_ids_one_by_one(self):
        def send_next_task_id():
            if self.task_ids:
                # 한 개의 task_id를 꺼내서 전송
                task_id = self.task_ids.pop(0)

                temp = [
                    {
                        'request': {
                            "fleet_name": self.fleet_name,
                            "robot_name": self.robot_name,
                            "type": "cancel_task_request",
                            "task_id": task_id
                        }
                    }
                ]
                formatted_data = json.dumps(temp)
                info(f'formatted_data: {formatted_data}')

                if self.connection_open:
                    try:
                        self.ws.send(formatted_data)
                        info(f"Sent task_id: {formatted_data}")
                    except Exception as e:
                        error(f"Failed to send task_id: {e}")

                # 아직 task_ids가 남아있다면 1초 후 다음 task_id 전송
                if self.task_ids:
                    threading.Timer(1.0, send_next_task_id).start()
            else:
                info("All task_ids have been sent.")

        # 첫 번째 실행 시작
        send_next_task_id()

    def rmf_patrol_request_callback(self, msg):
        parts = msg.data.split(',')
        if len(parts) < 3:
            print(f"Invalid message format: {msg.data}")
            return
        # 🔹 파싱된 데이터 할당
        task_type, category, value = parts
        value = value.replace(' ', ',')

        if task_type == "robot_task_request":
            temp = [
                {
                    'request': {
                        "fleet_name": self.fleet_name,
                        "robot_name": self.robot_name,
                        "type": task_type,
                        "category": category,
                        "value": value
                    }
                }
            ]

            formatted_data = json.dumps(temp)
            info(f'formatted_data: {formatted_data}')
            if self.connection_open:
                try:
                    self.ws.send(formatted_data)
                    info(f"Sent data: {formatted_data}")
                except Exception as e:
                    error(f"Failed to send data: {e}")
            else:
                error("Cannot send data: WebSocket is not connected.")

        elif task_type == "cancel_task_request":
            self.send_task_ids_one_by_one()

    def navigate(self, robot_name, cmd_id, dest, map_name, speed_limit):
        info(f'Navigate: {robot_name}, cmd_id: {cmd_id}, '
             f'x: {dest["x"]:.3f}, '
             f'y: {dest["y"]:.3f}, '
             f'yaw: {dest["yaw"]:.3f}'
             )
        response = {'success': False, 'msg': ''}
        if (robot_name not in self.robots or len(dest) < 1):
            return response

        # RMFGoTOPose에서 변경
        # target_x_rmf = dest.destination['x']
        # target_y_rmf = dest.destination['y']
        # target_x, target_y = rmf_to_robot_transform(target_x_rmf, target_y_rmf, self.reference_coordinates)

        target_x = dest['x']
        target_y = dest['y']
        target_yaw = dest['yaw']
        target_map = map_name
        target_speed_limit = speed_limit

        # target_x -= self.offset[0]
        # target_y -= self.offset[1]
        debug(f'target_x, target_y:{target_x},{target_y}')

        t = self.node.get_clock().now().to_msg()

        path_request = PathRequest()
        robot = self.robots[robot_name]
        if robot.state is None:
            return response

        cur_x = robot.state.location.x
        cur_y = robot.state.location.y
        cur_yaw = robot.state.location.yaw
        cur_loc = robot.state.location
        path_request.path.append(cur_loc)

        disp = self.disp([target_x, target_y], [cur_x, cur_y])
        duration = int(disp / self.vehicle_traits.linear.nominal_velocity) + \
                   int(abs(abs(cur_yaw) - abs(target_yaw)) /
                       self.vehicle_traits.rotational.nominal_velocity)
        t.sec = t.sec + duration
        target_loc = Location()
        target_loc.t = t
        target_loc.x = target_x
        target_loc.y = target_y
        target_loc.yaw = target_yaw
        target_loc.level_name = target_map
        target_loc.obey_approach_speed_limit = False
        if target_speed_limit is not None and target_speed_limit > 0.0:
            target_loc.obey_approach_speed_limit = True
            target_loc.approach_speed_limit = target_speed_limit

        path_request.fleet_name = self.fleet_name
        path_request.robot_name = robot_name
        path_request.path.append(target_loc)
        path_request.task_id = str(cmd_id)
        self.path_pub.publish(path_request)

        if self.debug:
            print(f'Sending navigate request for {robot_name}: {cmd_id}')
        robot.last_path_request = path_request
        robot.destination = target_loc

        response['success'] = True
        return response

    def stop_robot(self, robot_name, cmd_id):
        debug(f"stop_robot: {robot_name}, {cmd_id}")
        response = {'success': False, 'msg': ''}
        if robot_name not in self.robots:
            return response

        robot = self.robots[robot_name]
        if robot.state is None:
            return response

        path_request = PathRequest()
        path_request.fleet_name = self.fleet_name
        path_request.robot_name = robot_name
        path_request.path = []
        # Appending the current location twice will effectively tell the
        # robot to stop
        path_request.path.append(robot.state.location)
        path_request.path.append(robot.state.location)

        path_request.task_id = str(cmd_id)
        self.path_pub.publish(path_request)

        info(f'Sending stop request for {robot_name}: {cmd_id}')
        robot.last_path_request = path_request
        robot.destination = None

        response['success'] = True
        return response
