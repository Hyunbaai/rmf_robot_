# -*- coding: utf-8 -*-

import socketio
import uvicorn
from fastapi import FastAPI
import json

# FastAPI �ν��Ͻ� ����
app = FastAPI()

# Socket.IO ���� ����
sio = socketio.AsyncServer(
    cors_allowed_origins="*",  # CORS ���� ����
    async_mode="asgi",
    logger=True
)

# �ùٸ��� Socket.IO�� FastAPI�� ����Ʈ (��� ����)
app.mount("/socket.io", socketio.ASGIApp(sio, other_asgi_app=None))

# WebSocket �̺�Ʈ �ڵ鷯
@sio.on("gps")
async def handle_gps(sid, data):
    gps_data = json.loads(data)
    print(f"?? ���� GPS ������: {gps_data}")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
