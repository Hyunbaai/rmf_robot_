import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import asyncio
import websockets
from geometry_msgs.msg import PointStamped
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from std_msgs.msg import Int32, String
import json

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml string
    """

    def __init__(self, data: str) -> None:
        # JSON 형식으로 먼저 로드 시도, 실패하면 YAML 형식으로 처리
        try:
            self.wps_dict = json.loads(data)
            print("JSON 형식의 데이터로 처리됩니다.")
        except json.JSONDecodeError:
            self.wps_dict = yaml.safe_load(data)
            print("YAML 형식의 데이터로 처리됩니다.")

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml data
        """
        gepose_wps = []
        waypoints = self.wps_dict.get("waypoints", [])

        # Waypoints 데이터를 GeoPose 형식으로 변환하여 리스트에 추가
        for wp in waypoints:
            if "latitude" in wp and "longitude" in wp:
                # JSON 구조의 경우
                latitude = wp["latitude"]
                longitude = wp["longitude"]
                yaw = wp["heading"]  # JSON 구조에 yaw 필드가 없는 경우 기본값 사용
            elif "latlng" in wp:
                # YAML 구조의 경우
                latitude = wp["latlng"]["lat"]
                longitude = wp["latlng"]["lng"]
                yaw = wp.get("heading", 0.0)  # YAML 구조에서는 heading을 사용

            # GeoPose로 변환하여 리스트에 추가
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        
        return gepose_wps


class WebSocketGpsWpCommander(Node):
    """
    ROS2 Node to manage waypoint following using nav2 and receive waypoints via WebSocket
    """

    def __init__(self):
        super().__init__('websocket_gps_wp_commander')
        self.navigator = BasicNavigator()
        self.waypoint_count_pub = self.create_publisher(Int32, '/waypoint_count', 10)
        self.arrival_pub = self.create_publisher(String, '/arrival_status', 10)
        self.get_logger().info("Initialized WebSocket GPS Waypoint Commander Node.")

    async def websocket_handler(self, websocket, path):
        self.get_logger().info("WebSocket connection established. Awaiting YAML data...")
        try:
            while True:
                message = await websocket.recv()
                self.get_logger().info("YAML data received.")
                # Parse the received YAML data
                wp_parser = YamlWaypointParser(message)
                wps = wp_parser.get_wps()

                # Start waypoint following
                self.navigator.waitUntilNav2Active(localizer='robot_localization')
                self.navigator.followGpsWaypoints(wps)
                
                # Publish the waypoint count (over 1 in this case)
                self.publish_waypoint_count(len(wps))
                
                # Wait for the task to complete
                while not self.navigator.isTaskComplete():
                    await asyncio.sleep(0.1)
                
                self.get_logger().info("Waypoints completed successfully.")
                await websocket.send("Waypoints completed successfully.")
                self.publish_arrival_status("Arrived")
                self.get_logger().info("Ready for new YAML data...")

        except websockets.ConnectionClosed as e:
            self.get_logger().warn(f"WebSocket connection closed: {e}")

    def start_websocket_server(self, port=8000):
        self.get_logger().info(f"Starting WebSocket server on ws://192.168.0.10:{port}")
        # Run the WebSocket server
        start_server = websockets.serve(self.websocket_handler, "192.168.0.10", port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
        
    def publish_waypoint_count(self, count):
        msg = Int32()
        msg.data = count
        self.waypoint_count_pub.publish(msg)
        self.get_logger().info(f"Publish waypoint count: {count}")

    def publish_arrival_status(self, message):
        msg = String()
        msg.data = message
        self.arrival_pub.publish(msg)
        self.get_logger().info(f"Publish arrival status: {message}")

def main():
    rclpy.init()

    # Initialize the WebSocketGpsWpCommander node
    websocket_gps_pt_controller = WebSocketGpsWpCommander()

    # Start the WebSocket server
    websocket_gps_pt_controller.start_websocket_server()

    # Spin the ROS 2 node
    rclpy.spin(websocket_gps_pt_controller)


if __name__ == "__main__":
    main()