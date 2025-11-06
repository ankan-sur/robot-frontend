import os
import math
from typing import Tuple
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Request, Header
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any

# The ROS2 manager encapsulates rclpy init/shutdown and spins the node in a background thread
from ros_interface.ros_node import ROS2Manager


class MoveCommand(BaseModel):
    direction: str = Field(description="Movement direction, e.g., 'forward', 'backward', 'left', 'right'")
    speed: Optional[float] = Field(default=0.5, ge=0.0, le=1.0, description="Normalized speed [0.0, 1.0]")


app = FastAPI(title="HFH Robot Backend", version="0.1.0")

# Allow the frontend dev server to connect (adjust origins as needed)
# Includes robot hotspot network (192.168.149.x) and localhost for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "http://127.0.0.1:5173",
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        # Allow connections from robot hotspot network (192.168.149.x)
        "http://192.168.149.1:5173",
        "http://192.168.149.1:3000",
        # Allow any IP in hotspot range (for flexibility)
        # Note: In production, you may want to restrict this
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
def on_startup() -> None:
    """
    Start the ROS2 node and spin it in a background thread so FastAPI remains responsive.
    """
    try:
        ROS2Manager.start()
    except Exception as exc:
        raise RuntimeError(f"Failed to start ROS2 manager: {exc}")


@app.on_event("shutdown")
def on_shutdown() -> None:
    """
    Cleanly stop spinning and shutdown rclpy on server shutdown/reload.
    """
    ROS2Manager.stop()


@app.post("/move")
def move(cmd: MoveCommand, request: Request, authorization: str | None = Header(default=None)) -> Dict[str, Any]:
    """
    Command the robot to move. For now, this publishes a dummy message via ROS2 and
    returns static telemetry.
    """
    node = ROS2Manager.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    # Access control: require token if configured, and enforce lab proximity
    if not _validate_token(authorization):
        raise HTTPException(status_code=403, detail="Invalid or missing token")

    telemetry = node.get_status()
    allowed, reason = _control_allowed(request.client.host if request.client else None, telemetry)
    if not allowed:
        raise HTTPException(status_code=403, detail=f"Control not allowed: {reason}")

    # Publish a command on /hiwonder_cmd so you can debug with:
    #   ros2 topic echo /hiwonder_cmd
    node.publish_hiwonder_command(f"move:{cmd.direction}:{(cmd.speed or 0.5):.2f}")
    return {"ok": True, "telemetry": node.get_status()}


@app.post("/stop")
def stop() -> Dict[str, Any]:
    """
    Command the robot to stop. This publishes a dummy stop message via ROS2 and
    returns static telemetry.
    """
    node = ROS2Manager.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    # Emergency stop is always allowed regardless of access checks
    node.publish_hiwonder_command("stop")
    return {"ok": True, "telemetry": node.get_status()}


@app.get("/status")
def status() -> Dict[str, Any]:
    """
    Return current (dummy) telemetry from the ROS2 node.
    """
    node = ROS2Manager.get_node()
    if node is None:
        # Expose a reasonable fallback if ROS is still initializing
        return {
            "ok": False,
            "telemetry": {
                "battery": 98.0,
                "temperature_c": 43.2,
                "is_moving": False,
                "last_command": "unavailable",
            },
        }
    # Include control gate info
    telemetry = node.get_status()
    allowed, reason = _control_allowed(None, telemetry)
    telemetry.update({"control_allowed": allowed, "control_reason": reason})
    return {"ok": True, "telemetry": telemetry}


@app.websocket("/ws/telemetry")
async def telemetry_ws(ws: WebSocket) -> None:
    """
    WebSocket streaming of telemetry. The client connects to /ws/telemetry and
    receives a JSON telemetry object periodically. Useful for live dashboards.
    """
    await ws.accept()
    try:
        import asyncio

        while True:
            node = ROS2Manager.get_node()
            payload = {"ok": False, "telemetry": {}}
            if node is not None:
                telemetry = node.get_status()
                allowed, reason = _control_allowed(None, telemetry)
                telemetry.update({"control_allowed": allowed, "control_reason": reason})
                payload = {"ok": True, "telemetry": telemetry}
            await ws.send_json(payload)
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        return


# -------------------------
# Access Control helpers
# -------------------------

# Define known labs with approximate coordinates in odom/map frame (meters)
LABS = [
    {"name": "LabA", "x": 0.0, "y": 0.0},
    {"name": "LabB", "x": 10.0, "y": 0.0},
]

CONTROL_TOKEN = os.getenv("CONTROL_TOKEN")  # optional


def _nearest_lab_by_pose(x: float, y: float) -> Tuple[str, float]:
    best = ("", float("inf"))
    for lab in LABS:
        dx = x - float(lab["x"])
        dy = y - float(lab["y"])
        d = math.hypot(dx, dy)
        if d < best[1]:
            best = (str(lab["name"]), d)
    return best


def _client_lab_from_ip(ip: str | None) -> str:
    # Simple heuristic: map private subnets to labs; customize as needed
    if not ip:
        return "Unknown"
    if ip.startswith("192.168.1."):
        return "LabA"
    if ip.startswith("192.168.2."):
        return "LabB"
    return "Unknown"


def _control_allowed(client_ip: str | None, telemetry: Dict[str, Any]) -> Tuple[bool, str]:
    odom = telemetry.get("odom", {}) if telemetry else {}
    x = float(odom.get("x", 0.0))
    y = float(odom.get("y", 0.0))
    nearest_lab, distance = _nearest_lab_by_pose(x, y)
    client_lab = _client_lab_from_ip(client_ip)

    # If we cannot identify client lab, allow only if within 2m of any lab
    if client_lab == "Unknown":
        if distance <= 2.0:
            return True, f"Near {nearest_lab} (<=2m)"
        return False, "Client lab unknown and robot not near a lab"

    if client_lab != nearest_lab:
        return False, f"Client at {client_lab}, robot near {nearest_lab}"

    return True, f"Client and robot both at {nearest_lab}"


def _validate_token(header_token: str | None) -> bool:
    if not CONTROL_TOKEN:
        return True
    if not header_token:
        return False
    bearer = header_token
    if bearer.lower().startswith("bearer "):
        bearer = bearer[7:]
    return bearer == CONTROL_TOKEN


# Run with: uvicorn backend.main:app --host 0.0.0.0 --port 8000

