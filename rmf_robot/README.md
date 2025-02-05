## build
```bash
docker compose build iron
docker compose build rmf
```
## usage
```bash
xhost +
touch .env
다음과 같이 작성 123은 예시임
ROS_DOMAIN_ID=123
DISPLAY=0
CONFIG_FILE=/root/ws/src/rmf_demos/config/office/tinyRobot_with_nav2_config.yaml  <= 중요파일


# debugging 하려면 각각 실행 
# 전달된 models.tar.gz를 rmf_robot 폴더에 위치
docker compose up sim nav2 fsm

```
## etc
- 새로운 맵을 만드는 방법
  - traffic_editor로 vector, floor 등 만든다
  - rmf_demos_maps/maps/ 원하는 폴더를 만들고 *.building.yaml 저장위치
  - rmf_demos_dashboard_resources/ 원하는 폴더와 dashboard_config.json, main.json을 타 폴더내용 참고해서 작성할것 후에 rmf-panel-js에서 사용함
  - rmf_demos/config/ 원하는 폴더에 *_config.yaml 파일작성
  - rmf_demos/launch/ *.launch.xml 파일작성