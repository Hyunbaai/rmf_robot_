import asyncio
import threading
import websockets
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

# WebSocket 서버 포트
WEBSOCKET_SERVER_PORT = 9000

class WebSocketGpsIAViewer(Node):
    def __init__(self):
        super().__init__('websocket_gps_ia_viewer')

        # WebSocket 클라이언트 관리
        self.client = None
        self.client_connected = asyncio.Event()

        # ROS 2 구독자 설정
        self.create_subscription(String, '/status_data', self.status_data_callback, 10)

        # asyncio 이벤트 루프 설정
        self.loop = asyncio.get_event_loop()

    def status_data_callback(self, msg):
        if self.client:
            try:
                # 데이터 직렬화
                data = json.dumps({"status_data": msg.data})
                # 비동기 메소드 호출을 예약합니다.
                asyncio.run_coroutine_threadsafe(self.send_data(data), self.loop)
            except Exception as e:
                self.get_logger().error(f"Error sending data: {e}")

    async def send_data(self, data):
        if self.client:
            try:
                await self.client.send(data)
                self.get_logger().info(f"Sent data to client: {data}")
            except Exception as e:
                self.get_logger().error(f"Error sending data: {e}")

    async def register(self, websocket):
        self.client = websocket
        self.client_connected.set()  # 클라이언트 연결됨을 알림
        try:
            await websocket.wait_closed()
        finally:
            self.client = None
            self.client_connected.clear()  # 클라이언트 연결 끊어짐

async def start_websocket_server(node):
    async with websockets.serve(node.register, "192.168.0.10", WEBSOCKET_SERVER_PORT):
        await asyncio.Future()  # Run forever

def spin_ros_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketGpsIAViewer()

    # Create an asyncio event loop
    loop = asyncio.get_event_loop()

    try:
        # Run ROS 2 spin and WebSocket server concurrently
        # Use threading to run ROS 2 spin in a separate thread
        def run_ros():
            spin_ros_node(node)

        # Start ROS 2 spin in a separate thread
        ros_thread = threading.Thread(target=run_ros)
        ros_thread.start()

        # Run the WebSocket server in the main thread
        loop.run_until_complete(start_websocket_server(node))
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()

if __name__ == '__main__':
    main()