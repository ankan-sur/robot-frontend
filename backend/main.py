import asyncio
import os
import signal
from dataclasses import dataclass
from typing import Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel


BASE_ENV_SOURCE = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
DOCKER_CONTAINER = os.environ.get("ROS_DOCKER_CONTAINER")  # e.g., MentorPi
ROS_SETUP_IN_CONTAINER = os.environ.get("ROS_SETUP_IN_CONTAINER", "/opt/ros/humble/setup.bash")

BASE_BRINGUP_CMD = os.environ.get("BASE_BRINGUP_CMD", "ros2 launch bringup bringup.launch.py")
SLAM_MODE_CMD = os.environ.get("SLAM_MODE_CMD", "ros2 launch slam slam_mode.launch.py")
LOCALIZATION_MODE_CMD = os.environ.get("LOCALIZATION_MODE_CMD", "ros2 launch nav2 localization_mode.launch.py")


def ros_shell(cmd: str) -> str:
    """Build a shell command that runs a ROS CLI command either on host or inside a Docker container.

    If ROS_DOCKER_CONTAINER is set, we exec into that container and source ROS there.
    Otherwise, we source ROS on the host and run the command directly.
    """
    if DOCKER_CONTAINER:
        # Exec into the ROS container and run with its environment
        inner = f"source {ROS_SETUP_IN_CONTAINER} >/dev/null 2>&1 && {cmd}"
        return (
            "bash -lc '"
            f"docker exec -i {DOCKER_CONTAINER} bash -lc \"{inner}\""
            "'"
        )
    # Host mode: source host ROS then run
    return f"bash -lc 'source {BASE_ENV_SOURCE} >/dev/null 2>&1 && {cmd}'"


@dataclass
class ManagedProc:
    name: str
    cmd: str
    proc: Optional[asyncio.subprocess.Process] = None

    async def start(self):
        if self.proc and self.proc.returncode is None:
            return
        self.proc = await asyncio.create_subprocess_shell(
            ros_shell(self.cmd),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.returncode is None

    async def stop(self):
        if not self.proc:
            return
        if self.proc.returncode is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(self.proc.wait(), timeout=10)
            except asyncio.TimeoutError:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
        self.proc = None


class ModeState(str):
    IDLE = "IDLE"
    SLAM = "SLAM"
    LOCALIZATION = "LOCALIZATION"


class ModeManager:
    def __init__(self):
        self.slam = ManagedProc("slam", SLAM_MODE_CMD)
        self.localization = ManagedProc("localization", LOCALIZATION_MODE_CMD)
        self.state = ModeState.IDLE

    async def start_slam(self):
        if self.localization.is_running():
            raise HTTPException(409, detail="Localization mode active; stop it first")
        if not self.slam.is_running():
            await self.slam.start()
        self.state = ModeState.SLAM

    async def stop_slam(self):
        if self.slam.is_running():
            await self.slam.stop()
        if not self.localization.is_running():
            self.state = ModeState.IDLE

    async def start_localization(self):
        if self.slam.is_running():
            raise HTTPException(409, detail="SLAM mode active; stop it first")
        if not self.localization.is_running():
            await self.localization.start()
        self.state = ModeState.LOCALIZATION

    async def stop_localization(self):
        if self.localization.is_running():
            await self.localization.stop()
        if not self.slam.is_running():
            self.state = ModeState.IDLE

    def snapshot(self):
        return {
            "state": self.state,
            "slam_running": self.slam.is_running(),
            "localization_running": self.localization.is_running(),
        }


manager = ModeManager()
app = FastAPI(title="MentorPi Mode Server", version="0.1.0")


class SaveMapRequest(BaseModel):
    name: Optional[str] = None


@app.get("/mode/status")
async def mode_status():
    return manager.snapshot()


@app.post("/mode/slam/start")
async def start_slam():
    await manager.start_slam()
    return manager.snapshot()


@app.post("/mode/slam/save_exit")
async def save_map_and_exit(req: SaveMapRequest):
    if not manager.slam.is_running():
        raise HTTPException(409, detail="SLAM is not running")
    name = req.name or asyncio.get_event_loop().time().__str__().replace(".", "")
    # Use ros2 service call directly
    cmd = ros_shell(f"ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap '{{name: \"{name}\"}}'")
    proc = await asyncio.create_subprocess_shell(cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    out = (await proc.communicate())[0].decode(errors="ignore")
    if proc.returncode != 0:
        raise HTTPException(500, detail=f"SaveMap failed (rc={proc.returncode}): {out}")
    # Stop SLAM after save
    await manager.stop_slam()
    return {"ok": True, "saved": name, **manager.snapshot(), "log": out}


@app.post("/mode/slam/stop")
async def stop_slam():
    await manager.stop_slam()
    return manager.snapshot()


@app.post("/mode/localization/start")
async def start_localization():
    await manager.start_localization()
    return manager.snapshot()


@app.post("/mode/localization/stop")
async def stop_localization():
    await manager.stop_localization()
    return manager.snapshot()


@app.post("/maps/reload")
async def reload_maps():
    # Best effort: ping system_topics info to refresh internals; map name empty implies current/scan
    cmd = ros_shell("ros2 service call /system/map/info interfaces/srv/SetString '{}' || true")
    proc = await asyncio.create_subprocess_shell(cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    out = (await proc.communicate())[0].decode(errors="ignore")
    return {"ok": True, "log": out}
