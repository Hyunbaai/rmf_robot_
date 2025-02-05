#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import websocket  # websocket-client 패키지가 필요함
from sensor_msgs.msg import NavSatFix

# WebSocket 서버 주소 (Fleet Manager의 WebSocket 서버)
WEBSOCKET_URL = "ws://0.0.0.0:8080"

class GpsWebSocketPublisher(Node):
    def __init__(self):
        super().__init__('gps_websocket_publisher')
        
        # GPS 데이터 구독자 설정
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        self.websocket = None
        self.connect_websocket()

    def connect_websocket(self):
        """WebSocket 연결을 설정하는 함수"""
        try:
            self.websocket = websocket.WebSocket()
            self.websocket.connect(WEBSOCKET_URL)
            self.get_logger().info(f"Connected to WebSocket server: {WEBSOCKET_URL}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to WebSocket: {e}")
            self.websocket = None

    def gps_callback(self, msg: NavSatFix):
        """GPS 데이터를 수신하여 WebSocket을 통해 전송하는 콜백 함수"""
        if self.websocket is None:
            self.connect_websocket()
            if self.websocket is None:
                self.get_logger().error("Cannot send GPS data: WebSocket is not connected")
                return
        
        # JSON 데이터 생성
        gps_data = {
            "robot_id": "tinybot1",  # 로봇 ID (필요시 변경)
            "lat": msg.latitude,
            "lon": msg.longitude
        }

        # WebSocket을 통해 데이터 전송
        try:
            self.websocket.send(json.dumps(gps_data))
            self.get_logger().info(f"Sent GPS data: {gps_data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send GPS data via WebSocket: {e}")
            self.websocket = None  # 연결이 끊어졌다면 재연결을 시도하도록 설정

def main(args=None):
    rclpy.init(args=args)
    node = GpsWebSocketPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GPS WebSocket Publisher...")
    finally:
        if node.websocket:
            node.websocket.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
