## new build
```bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-$ROS_DISTRO-test-msgs
colcon build --packages-select \
  nav2_bringup \
  nav2_bt_navigator \
  nav2_behavior_tree \
  nav2_straightline_planner \
  rmf_demos_gz_classic

```
## new notebook install
- sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
- sudo apt install -y ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-navigation2
- sudo apt-get install ros-$ROS_DISTRO-tf-transformations
- pip3 install fastapi uvicorn websocket-client
- python3 -m pip install flask-socketio
- sudo apt update && sudo apt install ros-$ROS_DISTRO-rmf-dev
- 

## tf tree
- export ROS_DOMAIN_ID=7 && ros2 run tf2_tools view_frames
## 실행
- gpuserver or home
- nuc1
```bash
export ROS_DOMAIN_ID=7 && ros2 launch rmf_demos_gz_classic office_simulation.launch.xml headless:=False
export ROS_DOMAIN_ID=7 && ros2 launch rmf_demos_gz_classic office_with_nav2.launch.py headless:=False
export ROS_DOMAIN_ID=7 && ros2 run fsm_waypoint fsm_waypoint_node
export ROS_DOMAIN_ID=7 && ros2 run nav2_map_server map_saver_cli -f carto_battle_royale

```
- nuc2

## copy models
- tar -xzvf models.tar.gz -C /home/zeta/.gazebo
- tar -xzvf models.tar.gz -C /home/bcc/.gazebo

## map 생성 2가지
- cartographer_ros 사용 zeta2_edu_autonomous 참고
- png을 사용해서 traffic_editor에서 Scale -> yaml 해상도 변경

## map 변경
- rmf office.world -> nav2 turtlebot3 office.world로 변경
- office.png size: 3047x1717 -> 3047x0.05/2=76.175m, 1717x0.05/2=42.925m
- origin: [-76.175000, -42.925000, 0.000000]  # 좌하단 원점 설정

## 작업 사무실 변경시 수정사항
- fsm_waypoint_node.py에서 config_file 변경
- server에서 turtlebot3_world_config.yaml 변경 ip 주소 변경

## 로봇 변경시
- RMFNavToPose.py에서 self.robot_name 변경
- fsm_waypoint_node.py에서 initial_pose 변경
- rmf_demos/config/turtlebot3_world/turtlebot3_world_config.yaml에서 robot_name 변경
- RMFNavToPose.py에서 reference_coordinates 변경

## 메모리 확인
- ps aux --sort=-%mem | head -n 10

## robot#2
- ssh zeta@192.168.11.192
- 1

## usage
- git checkout tags/2.2.3 -b branch-2.2.3

```bash
export ROS_DOMAIN_ID=4 &&
ros2 launch rmf_demos_gz_classic robot.launch.xml headless:=True use_sim_time:=True

export ROS_DOMAIN_ID=4 &&
ros2 topic echo /robot_state
```
```yaml
# 0.yaml
    - - 10.433053704916215
      - -5.5750955876973505
      - {is_charger: true, is_holding_point: true, is_parking_spot: true, name: tinyRobot1_charger,
        spawn_robot_name: tinyRobot1, spawn_robot_type: TinyRobot}

    - - 20.423692180237488
      - -5.312098057266895
      - {is_charger: true, is_holding_point: true, is_parking_spot: true, name: tinyRobot2_charger,
        spawn_robot_name: tinyRobot2, spawn_robot_type: TinyRobot}
```

```yaml
# office.building.yaml

      - [1232.421, 658.567, 0, tinyRobot1_charger, {is_charger: [4, true], is_holding_point: [4, true], is_parking_spot: [4, true], spawn_robot_name: [1, tinyRobot1], spawn_robot_type: [1, TinyRobot]}]
      - [2412.581, 627.5, 0, tinyRobot2_charger, {is_charger: [4, true], is_holding_point: [4, true], is_parking_spot: [4, true], spawn_robot_name: [1, tinyRobot2], spawn_robot_type: [1, TinyRobot]}]

```

```yaml
 # office.world
    
    <rmf_charger_waypoints name="charger_waypoints">
      <rmf_vertex name="tinyRobot1_charger" x="10.433053704916215" y="-5.5750955876973505" level="L1" />
      <rmf_vertex name="tinyRobot2_charger" x="20.423692180237488" y="-5.312098057266895" level="L1" />
    </rmf_charger_waypoints>
```

## simulation
```bash
# home, gpuserver
export ROS_DOMAIN_ID=6 &&
export TURTLEBOT3_MODEL=waffle &&
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models &&
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

# home, gpuserver, map 변경
export ROS_DOMAIN_ID=6 &&
export TURTLEBOT3_MODEL=waffle &&
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models &&
ros2 launch rmf_demos_gz_classic office_with_nav2.launch.py headless:=False


# 

export ROS_DOMAIN_ID=6 && ros2 launch rmf_demos_gz_classic office_with_nav2.launch.py headless:=False
# office simulation
export ROS_DOMAIN_ID=6 && ros2 launch rmf_demos_gz_classic office_simulation.launch.xml headless:=False


# nuc
# robot1_charger
export ROS_DOMAIN_ID=7 &&
export TURTLEBOT3_MODEL=waffle &&
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models &&
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True \
  x_pose:=0.0824 y_pose:=0.5682 z_pose:=0.01 yaw:=1.57

# nuc 
export ROS_DOMAIN_ID=8 &&
export TURTLEBOT3_MODEL=waffle &&
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models &&
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True \
  x_pose:=0.0824 y_pose:=0.5682 z_pose:=0.01 yaw:=1.57

# nuc #2
export ROS_DOMAIN_ID=8 &&
ros2 run fsm_waypoint fsm_waypoint_node

# nuc #2
export ROS_DOMAIN_ID=8 &&
ros2 run rmf_demos_fleet_adapter test_path_request

# gpuserver or home
export ROS_DOMAIN_ID=6 &&
ros2 run fsm_waypoint fsm_waypoint_node

# nuc #1
export ROS_DOMAIN_ID=7 &&
ros2 run fsm_waypoint fsm_waypoint_node

# nuc #1
export ROS_DOMAIN_ID=7 &&
ros2 topic echo /robot_state

export ROS_DOMAIN_ID=6 &&
ros2 topic list

export ROS_DOMAIN_ID=6 &&
ros2 run rmf_demos_fleet_adapter test_path_request

export ROS_DOMAIN_ID=6 &&
ros2 run rmf_demos_fleet_adapter test_robot_state

export ROS_DOMAIN_ID=6 &&
ros2 bag play -l 

export ROS_DOMAIN_ID=6 && 
ros2 topic echo /clicked_point

export ROS_DOMAIN_ID=6 &&
ros2 topic echo /robot_path_request

export ROS_DOMAIN_ID=6 && 
ros2 run rmf_demos_fleet_adapter fleet_manager_with_fsm \
  -c /home/bcc/Works1/ws/install/rmf_demos/share/rmf_demos/config/turtlebot3_world/turtlebot3_world_config.yaml \
  -n dummy
  
export ROS_DOMAIN_ID=6 && 
ros2 run rmf_demos_fleet_adapter fleet_manager_with_fsm \
  -c /home/zeta/ws/install/rmf_demos/share/rmf_demos/config/turtlebot3_world/turtlebot3_world_config.yaml \
  -n dummy
  
  
export ROS_DOMAIN_ID=6 && ros2 launch zeta2_cartographer zeta2_cartographer.launch.py

```


## build
- install iron https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html
- install nav2 https://docs.nav2.org/getting_started/index.html#installation
- build fsm_waypoint, README.md 참고
- sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp