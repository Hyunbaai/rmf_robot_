FROM rmf_iron
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]


RUN sed -i 's|http://security.ubuntu.com/ubuntu|http://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list
RUN rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && apt-get update --fix-missing \
    && apt-get update \
    && apt-get upgrade -y --allow-downgrades --allow-remove-essential --allow-change-held-packages

## rolling에서는 안됨.
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-navigation2

## source 복사후 rosdep install로 의존성 설치하고 source 삭제
## 복사
WORKDIR /root/ws/src
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git --branch ros2

## 의존성 설치후 source 삭제
WORKDIR /root/ws
RUN apt-get update && apt-get upgrade -y && source /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/$ROS_DISTRO/setup.sh && colcon build

## 추가 의존성 설치
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-robot-localization  \
    ros-$ROS_DISTRO-joy  \
    ros-$ROS_DISTRO-teleop-twist-joy  \
    ros-$ROS_DISTRO-joint-state-publisher  \
    ros-$ROS_DISTRO-rviz2  \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-rmf-dev \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-test-msgs

## install smach
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install --upgrade pip &&  \
    pip install requests psutil py_trees pytz geographiclib supabase \
    fastapi \
    uvicorn \
    websocket-client \
    flask-socketio \
    flask-cors \
    websockets

WORKDIR /root/3rdparty
RUN git clone https://github.com/gabrielsr/standalone-smach.git standalone-smach && \
    sed -i '247s/isAlive()/is_alive()/g' standalone-smach/smach/concurrence.py && \
    cd standalone-smach && \
    python3 setup.py install

WORKDIR /root
COPY models.tar.gz /root/
RUN mkdir -p /root/.gazebo && \
    tar -xzvf models.tar.gz -C /root/.gazebo && \
    rm models.tar.gz

WORKDIR /root/ws/src
# nav2
COPY navigation2/nav2_bringup nav2_bringup
COPY navigation2/nav2_bt_navigator nav2_bt_navigator
COPY navigation2/nav2_behavior_tree nav2_behavior_tree
COPY navigation2_tutorials/nav2_straightline_planner nav2_straightline_planner
COPY navigation2_tutorials/nav2_gps_waypoint_follower_demo nav2_gps_waypoint_follower_demo
# fsm
COPY fsm_waypoint fsm_waypoint
# rmf
COPY rmf_demos rmf_demos
COPY rmf_demos_assets rmf_demos_assets
COPY rmf_demos_bridges rmf_demos_bridges
COPY rmf_demos_dashboard_resources rmf_demos_dashboard_resources
COPY rmf_demos_fleet_adapter rmf_demos_fleet_adapter
COPY rmf_demos_gz_classic rmf_demos_gz_classic
COPY rmf_demos_maps rmf_demos_maps
COPY rmf_demos_panel rmf_demos_panel
COPY rmf_demos_tasks rmf_demos_tasks


WORKDIR /root/ws
RUN apt-get update && source /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

RUN echo 'alias ls="ls --color=auto"' >> ~/.bashrc \
    && echo "source /root/ws/install/setup.bash" >> /root/.bashrc
