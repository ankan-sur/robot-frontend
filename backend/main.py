import os
import json
from pathlib import Path
from typing import Dict, Any
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio

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

async def _check_tcp_port(host: str, port: int, timeout: float = 0.5) -> bool:
    """Return True if a TCP connection can be established to host:port within timeout."""
    try:
        conn = asyncio.open_connection(host, port)
        reader, writer = await asyncio.wait_for(conn, timeout=timeout)
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass
        return True
    except Exception:
        return False

@app.get("/health")
async def health() -> Dict[str, Any]:
    """Overall system health check."""
    # Check rosbridge via TCP connect (rosbridge is WebSocket, not HTTP)
    rosbridge_ok = await _check_tcp_port("127.0.0.1", 9090, timeout=0.5)
    
    # Check video server
    video_ok = False
    try:
        import httpx
        async with httpx.AsyncClient() as client:
            resp = await client.get("http://127.0.0.1:8080/stream_viewer", timeout=1.0)
            video_ok = resp.status_code == 200
    except Exception:
        # Fallback to simple TCP probe if HTTP probe fails
        video_ok = await _check_tcp_port("127.0.0.1", 8080, timeout=0.5)
    
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

from fastapi.responses import StreamingResponse
import asyncio
from pathlib import Path

@app.get("/logs/stream")
async def stream_logs():
    """Stream log file content (for live log viewing)."""
    # Default log path - adjust as needed
    LOG_PATH = Path("log.txt")
    
    # If log file doesn't exist, return empty
    if not LOG_PATH.exists():
        async def empty_stream():
            yield "data: Log file not found\n\n"
        return StreamingResponse(empty_stream(), media_type="text/event-stream")
    
    async def log_stream():
        try:
            # Read last 100 lines initially, then stream new lines
            with open(LOG_PATH, 'r', encoding='utf-8', errors='ignore') as f:
                # Read all lines
                lines = f.readlines()
                # Send last 100 lines
                for line in lines[-100:]:
                    yield f"data: {line}\n\n"
                
                # Watch for new lines
                f.seek(0, 2)  # Seek to end
                while True:
                    line = f.readline()
                    if line:
                        yield f"data: {line}\n\n"
                    else:
                        await asyncio.sleep(0.1)  # Small delay when no new data
        except Exception as e:
            yield f"data: Error reading log: {str(e)}\n\n"
    
    return StreamingResponse(log_stream(), media_type="text/event-stream")

@app.get("/logs")
async def get_logs(limit: int = 500) -> Dict[str, Any]:
    """Get recent log entries."""
    LOG_PATH = Path("log.txt")
    if not LOG_PATH.exists():
        return {"logs": [], "error": "Log file not found"}
    
    try:
        with open(LOG_PATH, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
            recent_lines = lines[-limit:] if len(lines) > limit else lines
            return {"logs": recent_lines, "total_lines": len(lines)}
    except Exception as e:
        return {"logs": [], "error": str(e)}

# Optional: If you want to keep the /move and /stop endpoints for backward compatibility,
# you can use roslibpy to publish to /cmd_vel via rosbridge
# For now, these are removed since frontend communicates directly with ROS
