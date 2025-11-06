import os
import json
from pathlib import Path
from typing import Dict, Any
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# DO NOT import rclpy or ros_interface.ros_node
# ROS2 runs in Docker container, frontend connects via rosbridge
# If backend needs ROS access, use roslibpy instead (not rclpy)

app = FastAPI(title="HFH Robot Backend", version="0.2.0")

# CORS - allow all origins for development (adjust in production)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Config path
CONFIG_PATH = Path("/opt/robot/config.json")

@app.get("/health")
async def health() -> Dict[str, Any]:
    """Overall system health check."""
    # Check if rosbridge is accessible (it doesn't have HTTP endpoint, but we can check port)
    rosbridge_ok = False
    try:
        import httpx
        # Try to connect to rosbridge port (will fail, but tells us if port is open)
        async with httpx.AsyncClient() as client:
            try:
                await client.get("http://127.0.0.1:9090", timeout=0.5)
            except (httpx.ConnectError, httpx.TimeoutException):
                # Port exists but not HTTP - rosbridge is likely running
                rosbridge_ok = True
    except ImportError:
        # httpx not available
        pass
    except Exception:
        pass
    
    # Check video server
    video_ok = False
    try:
        import httpx
        async with httpx.AsyncClient() as client:
            resp = await client.get("http://127.0.0.1:8080/stream_viewer", timeout=1.0)
            video_ok = resp.status_code == 200
    except Exception:
        pass
    
    return {
        "ok": True,
        "rosbridge": {"available": rosbridge_ok, "port": 9090},
        "video_server": {"available": video_ok, "port": 8080},
    }

@app.get("/config")
async def get_config() -> Dict[str, Any]:
    """Read JSON config."""
    if CONFIG_PATH.exists():
        return json.loads(CONFIG_PATH.read_text())
    return {
        "speed_limits": {"linear": 0.5, "angular": 0.5},
        "topics": {
            "cmd_vel": "/cmd_vel",
            "odom": "/odom",
            "battery": "/ros_robot_controller/battery",
            "camera": "/ascamera/camera_publisher/rgb0/image"
        }
    }

@app.put("/config")
async def put_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Write JSON config."""
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(json.dumps(config, indent=2))
    return {"ok": True}

# Optional: If you want to keep the /move and /stop endpoints for backward compatibility,
# you can use roslibpy to publish to /cmd_vel via rosbridge
# For now, these are removed since frontend communicates directly with ROS
