import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class WebSocketGpsSpeedController(Node):
    """
    ROS2 Node to receive speed data via WebSocket and control robot's velocity
    """

    def __init__(self):
        super().__init__('websocket_gps_speed_controller')
        self.publisher = self.create_publisher(Float64, '/desired_speed', 10)
        self.get_logger().info("Initialized WebSocket GPS Speed Controller Node.")

    async def websocket_handler(self, websocket, path):
        self.get_logger().info("WebSocket connection established. Awaiting speed data...")
        try:
            while True:
                message = await websocket.recv()
                self.get_logger().info("Speed data received.")
                
                # Parse the received JSON data
                try:
                    data = json.loads(message)
                    speed = data.get('speed', 0)  # Default to 0 if 'speed' is not in the data
                except (json.JSONDecodeError, KeyError):
                    self.get_logger().error("Received invalid JSON data or missing 'speed' key.")
                    await websocket.send("Invalid data format. Expected JSON with 'speed' key.")
                    continue
                
                # Log received speed
                self.get_logger().info(f"Received speed: {speed}")

                # Create and publish Float64 message with received speed
                desired_speed_msg = Float64()
                desired_speed_msg.data = float(speed)
                self.publisher.publish(desired_speed_msg)
                
                await websocket.send("Speed applied successfully.")

        except websockets.ConnectionClosed as e:
            self.get_logger().warn(f"WebSocket connection closed: {e}")

    def start_websocket_server(self, port=7500):
        self.get_logger().info(f"Starting WebSocket server on ws://192.168.0.10:{port}")
        # Run the WebSocket server
        start_server = websockets.serve(self.websocket_handler, "192.168.0.10", port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()

def main():
    rclpy.init()

    # Initialize the WebSocketGpsSpeedController node
    speed_controller = WebSocketGpsSpeedController()

    # Start the WebSocket server
    speed_controller.start_websocket_server()

    # Spin the ROS 2 node
    rclpy.spin(speed_controller)

if __name__ == "__main__":
    main()

