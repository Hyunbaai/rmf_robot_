# -*- coding: utf-8 -*-

import socketio
import uvicorn
from fastapi import FastAPI
import json

# FastAPI ÀÎ½ºÅÏ½º »ý¼º
app = FastAPI()

# Socket.IO ¼­¹ö »ý¼º
sio = socketio.AsyncServer(
    cors_allowed_origins="*",  # CORS ¹®Á¦ ¹æÁö
    async_mode="asgi",
    logger=True
)

# ¿Ã¹Ù¸£°Ô Socket.IO¸¦ FastAPI¿¡ ¸¶¿îÆ® (Àç±Í ¹æÁö)
app.mount("/socket.io", socketio.ASGIApp(sio, other_asgi_app=None))

# WebSocket ÀÌº¥Æ® ÇÚµé·¯
@sio.on("gps")
async def handle_gps(sid, data):
    gps_data = json.loads(data)
    print(f"?? ¹ÞÀº GPS µ¥ÀÌÅÍ: {gps_data}")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
