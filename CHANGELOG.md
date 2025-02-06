## [1.1.0] - 2025-02-05

### 🛠 수정 (Fixed)
#### 🔹 Server
- `gl_deliveryRobot_config.yaml` 내 로봇 이름을 `seongsubot#`으로 변경

#### 🔹 Client
- `FleetManager.py`
  - `self.gps`값이 True로 변환 됨에 따라 사용하지 않는 서버 삭제
  - GPS 좌표 대신 `robot.state.location` 값을 사용하도록 변경  
  - `navigate()`시 위치 데이터 값 offset 중복 적용 삭제

#### **변경된 코드(FleetManager.py)**

```python
@self.sio.on("/gps")
def message(data):

    # if self.gps:
    #     while True:
    #         try:
    #             self.sio.connect('http://0.0.0.0:8080')
    #             break
    #         except Exception:
    #             self.node.get_logger().info(
    #                 f"Trying to connect to sio server at"
    #                 f"http://0.0.0.0:8080..")
    #             time.sleep(1)
```

```python
# 기존 코드 (GPS 좌표 사용)
def get_robot_state(self, robot: State, robot_name):
    if self.gps:
        position = copy.deepcopy(robot.gps_pos)
    else:
        position = [robot.state.location.x, robot.state.location.y]

# 변경 후 (항상 `robot.state.location` 사용)
def get_robot_state(self, robot: State, robot_name):
    position = [robot.state.location.x, robot.state.location.y]
```

```python
async def navigate(robot_name: str, cmd_id: int, dest: Request):

    # target_x -= self.offset[0]
    # target_y -= self.offset[1]
```

- `RMFNavToPose.py`
  - Action Client 변경 `navigate_to_pose` -> `follow_gps_waypoints`
  - `update_robot_state()`내에 `makeup_robot_state_publisher()` 호출하는 부분 주석

#### **변경된 코드(RMFNavToPose.py)**

```python
from nav2_msgs.action import NavigateToPose, FollowGPSWaypoints
self.action_client = ActionClient(self.node, FollowGPSWaypoints, 'follow_gps_waypoints')

def send_goal(self, destination_pose, task_id):
        
        if not isinstance(destination_pose, list):
            destination_pose = [destination_pose]

        goal_msg = FollowGPSWaypoints.Goal()
        
        waypoints = []
        for pose in destination_pose:
            geo_pose = GeoPose()
            geo_pose.position = GeoPoint(
                latitude=pose.position.latitude,
                longitude=pose.position.longitude,
                altitude=pose.position.altitude
            )
            geo_pose.orientation = pose.orientation  

            waypoints.append(geo_pose)

        goal_msg.gps_poses = waypoints

```

```python
def update_robot_state(self):
    # self.makeup_robot_state_publisher(rmf_x, rmf_y, target_yaw_rmf)
```


