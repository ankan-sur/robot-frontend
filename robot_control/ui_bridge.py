#!/usr/bin/env python3
import json, time, uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped

DEADMAN_S = 0.3
AUTO_TIMEOUT_S = 10.0

class UIBridge(Node):
    def __init__(self):
        super().__init__('ui_bridge')
        # pubs
        self.pub_state = self.create_publisher(String, '/robot/state', 10)
        self.pub_tel   = self.create_publisher(String, '/robot/telemetry', 10)
        self.pub_log   = self.create_publisher(String, '/robot/log', 50)
        self.pub_fb    = self.create_publisher(String, '/robot/cmd_feedback', 50)
        self.pub_posem = self.create_publisher(PoseStamped, '/robot/pose_map', 10)  # optional helper for UI
        # subs
        self.sub_cmd   = self.create_subscription(String, '/ui/cmd', self.on_cmd, 20)
        self.sub_cc    = self.create_subscription(String, '/ui/cmd_cancel', self.on_cancel, 10)
        self.sub_twist = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 50)
        self.sub_odom  = self.create_subscription(String, '/robot/odom_map_json', self.on_pose_map_json, 10)  # optional helper
        self.sub_batt  = self.create_subscription(Float32, '/ros_robot_controller/battery', self.on_batt, 10)

        self.queue = []
        self.current = None
        self.mode_manual = False
        self.last_twist_ts = 0.0
        self.last_manual_ts = 0.0
        self.battery = None
        self.pose_map = {"x":0.0, "y":0.0, "yaw":0.0}

        self.create_timer(0.1, self.tick_10hz)
        self.create_timer(1.0, self.tick_1hz)

    def log(self, s): self.pub_log.publish(String(data=s))

    def publish_state(self):
        d = {"state": "IDLE" if not self.current else "BUSY",
             "sub": "NONE",
             "mode": "MANUAL" if self.mode_manual else "AUTO"}
        self.pub_state.publish(String(data=json.dumps(d)))

    def fb(self, id_, status, msg="", progress=None):
        d = {"id": id_, "status": status, "message": msg}
        if progress is not None: d["progress"] = progress
        self.pub_fb.publish(String(data=json.dumps(d)))

    # ---- callbacks ----
    def on_cmd(self, m: String):
        try:
            cmd = json.loads(m.data)
            cmd.setdefault("id", str(uuid.uuid4()))
            self.queue.append(cmd)
            self.fb(cmd["id"], "QUEUED")
        except Exception as e:
            self.log(f"bad cmd json: {e}")

    def on_cancel(self, m: String):
        _id = json.loads(m.data)["id"]
        if self.current and self.current["id"] == _id:
            self.fb(_id, "CANCELED", "user cancel")
            self.current = None
        else:
            self.queue = [c for c in self.queue if c["id"] != _id]
            self.fb(_id, "CANCELED", "removed from queue")

    def on_twist(self, _: Twist):
        self.mode_manual = True
        self.last_twist_ts = time.time()
        self.last_manual_ts = self.last_twist_ts

    def on_batt(self, m: Float32):
        self.battery = float(m.data)

    def on_pose_map_json(self, m: String):
        # if a small helper node publishes base_link in map as JSON
        try:
            self.pose_map.update(json.loads(m.data))
        except Exception:
            pass

    # ---- timers ----
    def tick_10hz(self):
        t = time.time()
        # dead-man
        if self.mode_manual and (t - self.last_twist_ts) > DEADMAN_S:
            self.mode_manual = False
        # auto timeout back to AUTO
        if not self.mode_manual and (t - self.last_manual_ts) > AUTO_TIMEOUT_S and self.current is None:
            # already AUTO; nothing else
            pass
        # run queue
        if not self.mode_manual and self.current is None and self.queue:
            self.current = self.queue.pop(0)
            self.fb(self.current["id"], "RUNNING")
            self.log(f"RUN {self.current['type']}")
            # TODO: dispatch to real behaviors; here we simulate quick success
            self.fb(self.current["id"], "SUCCEEDED", "stub")
            self.current = None

    def tick_1hz(self):
        self.publish_state()
        tel = {"stamp": int(time.time()),
               "battery": self.battery,
               "pose_map": self.pose_map}
        self.pub_tel.publish(String(data=json.dumps(tel)))

def main():
    rclpy.init()
    n = UIBridge()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
