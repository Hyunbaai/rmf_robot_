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

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    DockSummary, RobotMode

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry

import numpy as np
from pyproj import Transformer

import socketio

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

import threading
import requests
from rmf_demos_fleet_adapter.utils.logger2 import info, warning, error, debug, critical

MODE_MAP = {
    0: "IDLE",
    1: "CHARGING",
    2: "MOVING",
    3: "PAUSED",
    4: "WAITING",
    5: "EMERGENCY",
    6: "GOING_HOME",
    7: "DOCKING",
    8: "ADAPTER_ERROR",
    9: "CLEANING",
}
app = FastAPI()


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


class FleetManager(Node):
    def __init__(self, config, nav_path):
        self.debug = False
        self.config = config
        self.fleet_name = self.config['rmf_fleet']['name']
        mgr_config = self.config['fleet_manager']

        self.gps = False
        self.offset = [0, 0]
        reference_coordinates_yaml = mgr_config.get('reference_coordinates')
        if reference_coordinates_yaml is not None:
            offset_yaml = reference_coordinates_yaml.get('offset')
            if offset_yaml is not None and len(offset_yaml) > 1:
                self.gps = True
                self.offset = offset_yaml

        super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.action_paths = {}  # Map activities to paths

        for robot_name, _ in self.config['rmf_fleet']['robots'].items():
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

        @self.sio.on("/gps")
        def message(data):
            try:
                robot = json.loads(data)
                robot_name = robot['robot_id']
                self.robots[robot_name].gps_to_xy(robot)
            except KeyError as e:
                info(f"Malformed GPS Message!: {e}")

        if self.gps:
            while True:
                try:
                    self.sio.connect('http://0.0.0.0:8080')
                    break
                except Exception:
                    warning(
                        f"Trying to connect to sio server at"
                        f"http://0.0.0.0:8080..")
                    time.sleep(1)

        self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_cb,
            100
        )

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default)

        @app.post('/open-rmf/rmf_demos_fm/pub_robot_path_requests/', response_model=Response)
        async def pub_robot_path_requests(data: dict):
            """
            Publishes a PathRequest to the robot.
            """
            try:
                # 데이터 검증
                if "fleet_name" not in data or "robot_name" not in data or "task_id" not in data or "path" not in data:
                    return {"success": False, "msg": "Missing required fields in the request"}

                path_request = PathRequest()
                path_request.fleet_name = data["fleet_name"]
                path_request.robot_name = data["robot_name"]
                path_request.task_id = data["task_id"]
                path_request.path = [
                    Location(
                        x=loc["x"],
                        y=loc["y"],
                        yaw=loc["yaw"],
                        level_name=loc.get("level_name", ""),
                        t=self.get_clock().now().to_msg(),
                    )
                    for loc in data.get("path", [])
                ]

                # Publish the PathRequest
                self.path_pub.publish(path_request)

                # 필요한 데이터를 소수점 한 자리로 반올림하여 추출
                summary = {
                    "name": data.get("robot_name"),
                    "task_id": data.get("task_id"),
                    "path": [
                        {
                            "x": round(p.get("x", 0), 2),
                            "y": round(p.get("y", 0), 2)
                        } for p in data.get("path", [])
                    ]
                }

                # path에서 최대 두 개의 요소만 출력
                path_summary = summary["path"][:2] if summary["path"] else []

                # 최종 메시지 포맷 지정
                log_message = (
                    f"name: {summary['name']}, "
                    f"task_id: {summary['task_id']}"
                )

                # path 정보 추가
                for i, path_point in enumerate(path_summary):
                    log_message += f", path[{i}]: x={path_point['x']}, y={path_point['y']}"

                # 로깅
                info(f'[path] server -> robot: {log_message}')
                return {"success": True, "msg": "Path request published successfully"}

            except KeyError as e:
                return {"success": False, "msg": f"Missing key: {e}"}

            except Exception as e:
                error(f"Error publishing PathRequest: {e}")
                return {"success": False, "msg": f"Error: {e}"}

    def robot_state_cb(self, msg):
        if (msg.name in self.robots):
            robot = self.robots[msg.name]
            if not robot.is_expected_task_id(msg.task_id) and \
                    not robot.mode_teleop:
                # This message is out of date, so disregard it.
                if robot.last_path_request is not None:
                    # Resend the latest task request for this robot, in case
                    # the message was dropped.

                    info(
                        f'Republishing task request for {msg.name}: '
                        f'{robot.last_path_request.task_id}, '
                        f'because it is currently following {msg.task_id}'
                    )
                    self.path_pub.publish(robot.last_path_request)
                error(f'Robot {msg.name} is not following the expected task_id')
                return

            robot.state = msg
            try:
                payload = {
                    "name": msg.name,
                    "task_id": str(msg.task_id),  # task_id는 문자열로 변환
                    "mode": {"mode": msg.mode.mode},
                    "path": [{"x": loc.x, "y": loc.y, "yaw": loc.yaw} for loc in msg.path],
                    "location": {
                        "x": msg.location.x,
                        "y": msg.location.y,
                        "yaw": msg.location.yaw,
                        "level_name": msg.location.level_name,
                    },
                    "battery_percent": msg.battery_percent,
                }
                url = "http://192.168.0.242:22011/open-rmf/rmf_demos_fm/sub_robot_state/"
                response = requests.post(url, json=payload)
                if response.status_code == 200:
                    # debug(f"Successfully forwarded state for {msg.name}")
                    pass
                else:
                    error(
                        f"Failed to forward state for {msg.name}: {response.text}"
                    )
                # debugging
                # info(f"robot_state_cb: {json.dumps(payload, indent=2)}")
                simplified_info = {
                    "name": payload["name"],
                    "task_id": payload["task_id"],
                    "mode": MODE_MAP.get(msg.mode.mode, f"UNKNOWN({msg.mode.mode})"),  # 간단히 mode 값만 가져옴
                    "location": {
                        "x": round(payload["location"]["x"], 2),
                        "y": round(payload["location"]["y"], 2),
                    },
                    "path": [
                        {
                            "x": round(payload["path"][0]["x"], 2),
                            "y": round(payload["path"][0]["y"], 2),
                        } if len(payload["path"]) > 0 else {},
                        {
                            "x": round(payload["path"][1]["x"], 2),
                            "y": round(payload["path"][1]["y"], 2),
                        } if len(payload["path"]) > 1 else {},
                    ],
                }

                # 간소화된 정보 출력
                log_message = (
                    f"name: {simplified_info['name']}, "
                    f"task_id: {simplified_info['task_id']}, "
                    f"mode: {simplified_info['mode']}, "
                    f"loc: x={simplified_info['location']['x']}, y={simplified_info['location']['y']}"
                )

                # path 정보 추가
                for i, path_point in enumerate(simplified_info["path"]):
                    if path_point:  # path가 비어있지 않은 경우만 추가
                        log_message += f", path[{i}]: x={path_point['x']}, y={path_point['y']}"

                debug(f'[state] slotcar -> robot: {log_message}')

            except Exception as e:
                warning(f"Error forwarding state for {msg.name}: {e}")

            # Check if robot has reached destination
            if robot.destination is None:
                return

            if (
                (
                    msg.mode.mode == RobotMode.MODE_IDLE
                    or msg.mode.mode == RobotMode.MODE_CHARGING
                )
                and len(msg.path) == 0
            ):
                robot = self.robots[msg.name]
                robot.destination = None
                completed_request = int(msg.task_id)
                if robot.last_completed_request != completed_request:
                    if self.debug:
                        print(
                            f'Detecting completed request for {msg.name}: '
                            f'{completed_request}'
                        )
                robot.last_completed_request = completed_request
                error(f'Robot {msg.name} has completed task {completed_request}')

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if (fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    args = parser.parse_args(args_without_ros[1:])
    info(f"Starting fleet manager...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config, args.nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(
        app,
        host=config['fleet_manager']['ip'],
        port=config['fleet_manager']['port'],
        log_level='warning'
    )


if __name__ == '__main__':
    main(sys.argv)
