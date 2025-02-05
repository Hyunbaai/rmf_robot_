import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import asyncio
import websockets
import json
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from std_msgs.msg import Int32, String

class WebSocketGpsPtController(Node):
    """
    ROS2 Node to receive GPS waypoints via WebSocket and send them to Nav2 for navigation
    """

    def __init__(self):
        super().__init__('websocket_gps_pt_controller')
        self.navigator = BasicNavigator()
        self.waypoint_count_pub = self.create_publisher(Int32, '/waypoint_count', 10)
        self.arrival_pub = self.create_publisher(String, '/arrival_status', 10)
        self.get_logger().info("Initialized WebSocket GPS Point Controller Node.")

    async def websocket_handler(self, websocket, path):
        self.get_logger().info("WebSocket connection established. Awaiting GPS data...")
        try:
            while True:
                message = await websocket.recv()
                self.get_logger().info("GPS data received.")
                
                # Parse the received JSON data
                try:
                    data = json.loads(message)
                    latitude = data['latitude']
                    longitude = data['longitude']
                    heading = data['heading']
                except (json.JSONDecodeError, KeyError):
                    self.get_logger().error("Received invalid JSON data or missing keys.")
                    await websocket.send("Invalid data format. Expected JSON with 'latitude' and 'longitude' keys.")
                    continue
                
                self.get_logger().info(f"Received GPS waypoint: lat={latitude}, lon={longitude}")

                # Convert to Geopose and send to Nav2
                self.navigator.waitUntilNav2Active(localizer='robot_localization')
                wp = [latLonYaw2Geopose(latitude, longitude, heading)]
                self.navigator.followGpsWaypoints(wp)

                # Publish the waypoint count (1 in this case)
                self.publish_waypoint_count(len(wp))
                
                # Wait for the task to complete
                while not self.navigator.isTaskComplete():
                    await asyncio.sleep(0.1)

                self.get_logger().info("Waypoint reached successfully.")
                await websocket.send("Waypoint reached successfully.")
                self.publish_arrival_status("Arrived")
                self.get_logger().info("Ready for new GPS data...")

        except websockets.ConnectionClosed as e:
            self.get_logger().warn(f"WebSocket connection closed: {e}")

    def start_websocket_server(self, port=7000):
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

    # Initialize the WebSocketGpsPtController node
    gps_wp_commander = WebSocketGpsPtController()

    # Start the WebSocket server
    gps_wp_commander.start_websocket_server()

    # Spin the ROS 2 node
    rclpy.spin(gps_wp_commander)

if __name__ == "__main__":
    main()
