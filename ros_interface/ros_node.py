import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

# Optional hardware bridge placeholder (not used yet)
from ros_interface.hiwonder_bridge import HiwonderBridge


class RobotROSNode(Node):
    """
    Minimal ROS2 node with dummy publishers/subscribers for Jetson/Raspberry Pi.
    - Publishes movement commands as String messages
    - Subscribes to a dummy telemetry topic and caches the last message
    """

    def __init__(self) -> None:
        super().__init__("robot_ros_node")

        # Publisher for Hiwonder command topic. Test with: ros2 topic echo /hiwonder_cmd
        self._hiwonder_cmd_pub = self.create_publisher(String, "/hiwonder_cmd", 10)

        # Subscribe to /hiwonder_cmd and execute corresponding bridge actions.
        self._hiwonder_cmd_sub = self.create_subscription(
            String, "/hiwonder_cmd", self._on_hiwonder_cmd, 10
        )

        self._last_telemetry: Optional[String] = None
        self._last_command: str = "idle"
        self._is_moving: bool = False

        # Placeholder for a hardware bridge (Hiwonder/servo controller, etc.)
        self._bridge = HiwonderBridge()

        self.get_logger().info("RobotROSNode initialized")

    # -------------------------
    # Subscriber callbacks
    # -------------------------
    def _on_telemetry(self, msg: String) -> None:
        self._last_telemetry = msg

    def _on_hiwonder_cmd(self, msg: String) -> None:
        """
        Parse simple text commands and invoke the Hiwonder bridge.
        Supported formats:
          - "move:forward:0.5"  -> direction, speed
          - "stop"
        """
        data = msg.data.strip().lower()
        try:
            if data.startswith("move:"):
                parts = data.split(":")
                direction = parts[1] if len(parts) > 1 else "forward"
                speed = float(parts[2]) if len(parts) > 2 else 0.5
                self._bridge.move_to_lab(direction=direction, speed=speed)
                self._is_moving = True
                self._last_command = data
            elif data == "stop":
                self._bridge.stop_motion()
                self._is_moving = False
                self._last_command = data
            else:
                self.get_logger().warn(f"Unknown hiwonder command: {data}")
        except Exception as exc:
            self.get_logger().error(f"Failed to execute command '{data}': {exc}")

    # -------------------------
    # Public API used by FastAPI
    # -------------------------
    def move(self, direction: str, speed: float) -> None:
        """
        Publish a dummy movement command. Replace with real message types and bridge calls.
        """
        payload = f"move:{direction}:{speed:.2f}"
        self._hiwonder_cmd_pub.publish(String(data=payload))
        self._last_command = payload
        self._is_moving = True
        self.get_logger().info(f"Published command: {payload}")

    def stop(self) -> None:
        """
        Publish a dummy stop command.
        """
        payload = "stop"
        self._hiwonder_cmd_pub.publish(String(data=payload))
        self._last_command = payload
        self._is_moving = False
        self.get_logger().info("Published command: stop")

    def get_status(self) -> Dict[str, Any]:
        """
        Return static/dummy telemetry for now.
        """
        telemetry_str = self._last_telemetry.data if self._last_telemetry else "{}"
        bridge_status = self._bridge.get_status()
        return {
            "battery": bridge_status.get("battery", 0.0),
            "odom": bridge_status.get("odom", {}),
            "is_moving": self._is_moving,
            "last_command": self._last_command,
            "last_raw_telemetry": telemetry_str,
            "sdk_available": bridge_status.get("sdk_available", False),
        }

    # -------------------------
    # Convenience for FastAPI
    # -------------------------
    def publish_hiwonder_command(self, command: str) -> None:
        self._hiwonder_cmd_pub.publish(String(data=command))


class ROS2Manager:
    """
    Manages rclpy lifecycle and spins the node in a background thread so the
    FastAPI process can handle HTTP requests concurrently.
    """

    _node: Optional[RobotROSNode] = None
    _executor: Optional[MultiThreadedExecutor] = None
    _thread: Optional[threading.Thread] = None
    _lock = threading.Lock()
    _started: bool = False

    @classmethod
    def start(cls) -> None:
        with cls._lock:
            if cls._started:
                return

            rclpy.init(args=None)
            cls._node = RobotROSNode()
            cls._executor = MultiThreadedExecutor()
            cls._executor.add_node(cls._node)

            # Spin in a daemon thread to avoid blocking the FastAPI server.
            def _spin() -> None:
                try:
                    cls._executor.spin()
                except Exception as exc:
                    # Log inside ROS2 node if available; otherwise print as fallback.
                    if cls._node is not None:
                        cls._node.get_logger().error(f"Executor spin error: {exc}")
                    else:
                        print(f"Executor spin error: {exc}")

            cls._thread = threading.Thread(target=_spin, name="ros2-executor-thread", daemon=True)
            cls._thread.start()
            cls._started = True

    @classmethod
    def stop(cls) -> None:
        with cls._lock:
            if not cls._started:
                return

            try:
                if cls._executor is not None:
                    cls._executor.shutdown()
                if cls._node is not None:
                    cls._node.destroy_node()
            finally:
                rclpy.shutdown()

            if cls._thread is not None and cls._thread.is_alive():
                cls._thread.join(timeout=2.0)

            cls._node = None
            cls._executor = None
            cls._thread = None
            cls._started = False

    @classmethod
    def get_node(cls) -> Optional[RobotROSNode]:
        return cls._node


