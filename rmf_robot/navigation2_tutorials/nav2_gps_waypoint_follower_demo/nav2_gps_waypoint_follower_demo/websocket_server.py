#!/usr/bin/env python3

import socketio
import uvicorn
from fastapi import FastAPI

# Create FastAPI app
app = FastAPI()

# Create Socket.IO server
sio = socketio.AsyncServer(
    cors_allowed_origins="*",
    async_mode="asgi",
    logger=True
)

# Mount Socket.IO to FastAPI
app.mount("/", socketio.ASGIApp(sio, other_asgi_app=app))

# Event: Client connected
@sio.event
async def connect(sid, environ):
    print(f"Client {sid} connected")

# Event: Client disconnected
@sio.event
async def disconnect(sid):
    print(f"Client {sid} disconnected")


def main():
    uvicorn.run(app, host="0.0.0.0", port=8080)

if __name__ == "__main__":
    main()