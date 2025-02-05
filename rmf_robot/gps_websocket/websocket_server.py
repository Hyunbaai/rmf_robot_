# -*- coding: utf-8 -*-

import socketio
import uvicorn
from fastapi import FastAPI
import json

# FastAPI 인스턴스 생성
app = FastAPI()

# Socket.IO 서버 생성
sio = socketio.AsyncServer(
    cors_allowed_origins="*",  # CORS 문제 방지
    async_mode="asgi",
    logger=True
)

# 올바르게 Socket.IO를 FastAPI에 마운트 (재귀 방지)
app.mount("/socket.io", socketio.ASGIApp(sio, other_asgi_app=None))

# WebSocket 이벤트 핸들러
@sio.on("gps")
async def handle_gps(sid, data):
    gps_data = json.loads(data)
    print(f"?? 받은 GPS 데이터: {gps_data}")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
