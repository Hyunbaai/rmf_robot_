#!/usr/bin/env bash

# ROS 2 환경 설정
source /root/ws/install/setup.bash

# 노드 확인을 위한 함수
check_node() {
    local node_name=$1
    local timeout=$2
    local end=$((SECONDS+timeout))

    while [ $SECONDS -lt $end ]; do
        if ros2 node list | grep -q $node_name; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# 노드 확인 타임아웃 설정 (초 단위)
TIMEOUT=30

# 확인해야 할 노드 목록
NODES=("waypoint_follower" "controller_server" "bt_navigator" "planner_server" "smoother_server")

# 모든 노드가 실행 중인지 확인
for node in "${NODES[@]}"; do
    echo "Checking if $node is active..."
    if ! check_node $node $TIMEOUT; then
        echo "$node not active within the timeout period."
        exit 1
    fi
    echo "$node is active."
done

echo "All required nodes are active. Running fsm_waypoint_node..."
ros2 run fsm_waypoint fsm_waypoint_node
