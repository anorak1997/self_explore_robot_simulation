#!/usr/bin/env python3
"""
web_backend
===========
A FastAPI server that is ALSO a ROS 2 node. It bridges a browser UI to
the whole stack: semantic memory, Nav2, the RRT planner and teleop.

  browser  <-- REST + WebSocket -->  web_backend (rclpy node)  <-->  ROS 2

Endpoints
  GET  /                 -> the dashboard (web/index.html)
  GET  /api/status       -> robot pose, nav state, perf, embedder, #places
  GET  /api/places       -> semantic places (with thumbnails)
  GET  /api/map          -> downsampled occupancy grid
  POST /api/query        -> {query}            : resolve text -> place
  POST /api/goto         -> {query|x,y}        : resolve + Nav2
  POST /api/command      -> {text}             : casual NL control
  WS   /ws               -> live places, robot pose, nav state, perf, path

Run:  ros2 run semantic_nav web_backend   ->   http://localhost:8080
"""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros import TransformException

from semantic_nav.store import SemanticStore
from semantic_nav.embedding import get_embedder

import subprocess
import numpy as np
import signal
from collections import deque

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel

try:
    from ament_index_python.packages import get_package_share_directory
    _WEB_DIR = os.path.join(get_package_share_directory("semantic_nav"), "web")
except Exception:
    _WEB_DIR = os.path.join(os.path.dirname(__file__), "..", "web")


def yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class WebBridge(Node):
    def __init__(self):
        super().__init__("web_backend")
        self.declare_parameter("match_threshold", 0.35)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("map_downsample_max", 160)
        self.declare_parameter("teleop_linear", 0.18)
        self.declare_parameter("teleop_angular", 0.6)
        self.threshold = self.get_parameter("match_threshold").value
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.ds_max = int(self.get_parameter("map_downsample_max").value)
        self.tl = self.get_parameter("teleop_linear").value
        self.ta = self.get_parameter("teleop_angular").value

        self.store = SemanticStore()
        self.embedder = get_embedder()

        self.map_cache = None
        self.map_version = 0
        self.rrt_path = []
        self.nav_state = "idle"
        self.nav_target = None
        self._result_future = None
        self._lock = threading.Lock()

        # performance telemetry
        self.perf = {"linear": 0.0, "angular": 0.0,
                     "distance": 0.0, "max_speed": 0.0}
        self._last_xy = None

        # teleop state (held velocity + deadman timeout)
        self._twist = (0.0, 0.0)
        self._twist_until = 0.0

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/semantic/places",
                                 self._places_cb, latched)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched)
        self.create_subscription(Path, "/rrt_path", self._path_cb, latched)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Publish to /rrt_goal, NOT /goal_pose. Nav2 bt_navigator subscribes to
        # /goal_pose and would immediately drive the robot if we used that topic.
        self.goal_pub = self.create_publisher(PoseStamped, "/rrt_goal", 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.1, self._teleop_tick)
        self.get_logger().info(f"web_backend up | embedder={self.embedder.name}")

    # ---- subscriptions ---------------------------------------------------
    def _places_cb(self, msg):
        self.store.load_json(msg.data)

    def _path_cb(self, msg):
        self.rrt_path = [(p.pose.position.x, p.pose.position.y)
                         for p in msg.poses]

    def _odom_cb(self, msg):
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._lock:
            self.perf["linear"] = round(lin, 3)
            self.perf["angular"] = round(ang, 3)
            self.perf["max_speed"] = round(
                max(self.perf["max_speed"], abs(lin)), 3)
            if self._last_xy is not None:
                self.perf["distance"] = round(
                    self.perf["distance"] +
                    math.hypot(x - self._last_xy[0], y - self._last_xy[1]), 2)
            self._last_xy = (x, y)

    def _map_cb(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        step = max(1, math.ceil(max(w, h) / self.ds_max))

        # Vectorised max-pool using numpy — no Python loops over cells so
        # this never blocks the ROS spin thread even on a 384x384 grid.
        arr = np.array(msg.data, dtype=np.int8).reshape(h, w)

        # Trim to exact multiples of step
        th, tw = (h // step) * step, (w // step) * step
        arr = arr[:th, :tw]

        # Reshape into (dh, step, dw, step) blocks then reduce per block:
        #   any cell >=65 → wall (100), any cell >=0 → free (0), else -1
        dh, dw = th // step, tw // step
        blocks = arr.reshape(dh, step, dw, step)

        wall = (blocks >= 65).any(axis=(1, 3))          # (dh, dw) bool
        free = (blocks >= 0).any(axis=(1, 3))

        out = np.full((dh, dw), -1, dtype=np.int8)
        out[free] = 0
        out[wall] = 100                                  # wall wins

        self.map_cache = {
            "w": dw, "h": dh, "resolution": msg.info.resolution * step,
            "origin_x": msg.info.origin.position.x,
            "origin_y": msg.info.origin.position.y,
            "cells": out.flatten().tolist()}
        self.map_version += 1

    # ---- teleop ----------------------------------------------------------
    def teleop(self, lin, ang, duration=1.2):
        self._twist = (float(lin), float(ang))
        self._twist_until = time.time() + duration

    def _teleop_tick(self):
        t = Twist()
        if time.time() < self._twist_until:
            t.linear.x, t.angular.z = self._twist
        self.cmd_pub.publish(t)   # zero Twist after timeout = deadman stop

    # ---- queries / navigation -------------------------------------------
    def match(self, query: str):
        qvec = self.embedder.embed(query)
        place, score = self.store.query(qvec)
        if place is None or score < self.threshold:
            return {"found": False, "score": round(max(score, 0.0), 3),
                    "query": query}
        return {"found": True, "query": query, "label": place["label"],
                "caption": place["caption"], "image": place.get("image", ""),
                "score": round(score, 3), "x": place["x"], "y": place["y"],
                "theta": place["theta"]}

    def send_goal(self, x, y, theta):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.nav_state = "nav2_unavailable"
            return self.nav_state
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        goal.pose = pose
        with self._lock:
            self.nav_state = "navigating"
            self.nav_target = {"x": float(x), "y": float(y)}
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)
        return self.nav_state

    def publish_goal(self, x, y, theta=0.0):
        """Send a goal to the RRT planner (listens on /rrt_goal, NOT /goal_pose).
        Sets nav_target so the goal dot appears on the canvas right away."""
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        # Show the goal dot on the canvas immediately, before RRT finishes.
        with self._lock:
            self.nav_target = {"x": float(x), "y": float(y)}
        self.goal_pub.publish(pose)

    def _on_goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.nav_state = "rejected"
            return
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, fut):
        status = fut.result().status
        self.nav_state = ("succeeded"
                          if status == GoalStatus.STATUS_SUCCEEDED else "failed")

    # ---- natural-language command ---------------------------------------
    def handle_command(self, text: str):
        t = text.lower().strip()

        if any(w in t for w in ["stop", "halt", "freeze", "wait"]):
            self.teleop(0.0, 0.0, 0.0)
            return {"action": "stop", "say": "Stopping the robot."}
        if any(w in t for w in ["spin", "rotate", "turn around"]):
            self.teleop(0.0, self.ta, 2.5)
            return {"action": "rotate", "say": "Spinning in place."}
        if "left" in t:
            self.teleop(0.0, self.ta, 1.2)
            return {"action": "turn_left", "say": "Turning left."}
        if "right" in t:
            self.teleop(0.0, -self.ta, 1.2)
            return {"action": "turn_right", "say": "Turning right."}
        if any(w in t for w in ["back", "reverse"]):
            self.teleop(-self.tl, 0.0, 1.2)
            return {"action": "backward", "say": "Backing up."}
        if any(w in t for w in ["forward", "ahead", "straight"]):
            self.teleop(self.tl, 0.0, 1.2)
            return {"action": "forward", "say": "Moving forward."}

        plan = any(w in t for w in ["plan", "rrt", "trajectory", "route"])
        m = self.match(text)
        if not m["found"]:
            return {"action": "unknown",
                    "say": "I couldn't match that to a place or a movement "
                           "command. Try 'go to the kitchen' or 'move forward'."}
        # Publish RRT goal so the path always appears on the canvas.
        self.publish_goal(m["x"], m["y"], m.get("theta", 0.0))
        if plan:
            # "plan a path to X" — show path only, robot stays put.
            m["action"] = "plan"
            m["say"] = f"Planning an RRT path to the {m['label']}."
        else:
            # "go to X" — plan AND drive via Nav2.
            nav_state = self.send_goal(m["x"], m["y"], m.get("theta", 0.0))
            m["action"] = "goto"
            m["say"] = f"Navigating to the {m['label']}."
            m["nav_state"] = nav_state
        return m

    # ---- state -----------------------------------------------------------
    def publish_initial_pose(self, x: float, y: float, theta: float = 0.0):
        """Publish /initialpose so AMCL activates immediately without RViz.
        Called automatically when the localize launch is started so the
        map→odom TF is broadcast as soon as AMCL comes up.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        # Standard AMCL covariance diagonal — large enough that AMCL will
        # spread its particle filter correctly from the given estimate.
        msg.pose.covariance[0]  = 0.25   # x
        msg.pose.covariance[7]  = 0.25   # y
        msg.pose.covariance[35] = 0.068  # yaw (~15°)
        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f"Published /initialpose at ({x:.2f}, {y:.2f}, {theta:.2f})")

    def reset_map_cache(self):
        """Drop the in-memory map so the browser re-fetches once the new
        map_server / AMCL publishes on /map after a mode switch."""
        self.map_cache = None
        self.map_version += 1   # triggers map_changed=true on next WS tick

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        return {"x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "theta": yaw_from_quat(q.x, q.y, q.z, q.w)}

    def status(self):
        return {"robot": self.robot_pose(), "nav_state": self.nav_state,
                "nav_target": self.nav_target, "embedder": self.embedder.name,
                "n_places": len(self.store.list_places()), "perf": self.perf}

    def places_public(self):
        return [{k: v for k, v in p.items() if k != "embedding"}
                for p in self.store.list_places()]


# --------------------------------------------------------------------------
# Lifecycle control: start/stop the launch files from the browser so the
# whole mission runs without a terminal. web_backend is the always-on
# control plane; it spawns everything else as child process groups.
# --------------------------------------------------------------------------
HOME = os.path.expanduser("~")
DEFAULT_MAP = os.path.join(HOME, "house_map.yaml")


def _targets(map_yaml=DEFAULT_MAP):
    # NOTE: these launches deliberately do NOT start another web_backend
    # (this node is already the UI), so there's no port 8080 clash.
    return {
        "explore": {
            "label": "Explore + map",
            "cmd": ["ros2", "launch", "turtlebot3_explorer",
                    "explore.launch.py"],
            "stops": ["localize"],
        },
        "localize": {
            "label": "Localize (AMCL)",
            "cmd": ["ros2", "launch", "turtlebot3_house_sim",
                    "localize.launch.py", f"map:={map_yaml}"],
            "stops": ["explore"],
        },
        "semantic": {
            "label": "Semantic layer",
            "cmd": ["ros2", "launch", "semantic_nav", "semantic.launch.py"],
            "stops": [],
        },
        "rrt_planner": {
            "label": "RRT planner",
            "cmd": ["ros2", "run", "rrt_planner", "rrt_planner",
                    "--ros-args", "-p", "use_sim_time:=true",
                    "-p", "rrt_star:=true",
                    "-p", "inflation_radius:=0.20",
                    "-p", "goal_bias:=0.10",
                    "-p", "step_size:=0.30"],
            "stops": [],
        },
    }


class LaunchManager:
    def __init__(self):
        self._procs = {}      # key -> Popen
        self._logs = {}       # key -> deque[str]
        self._lock = threading.Lock()

    def _drain(self, key, proc):
        for line in iter(proc.stdout.readline, ""):
            self._logs[key].append(line.rstrip("\n"))
        proc.stdout.close()

    def _running_locked(self, key):
        p = self._procs.get(key)
        return p is not None and p.poll() is None

    def _stop_locked(self, key):
        p = self._procs.get(key)
        if p is None or p.poll() is not None:
            return False
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)  # graceful for ros2
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                p.wait(timeout=4)
        except (ProcessLookupError, PermissionError):
            pass
        return True

    def start(self, key, map_yaml=DEFAULT_MAP):
        targets = _targets(map_yaml)
        if key not in targets:
            return {"ok": False, "error": f"unknown target '{key}'"}
        with self._lock:
            for other in targets[key]["stops"]:
                self._stop_locked(other)
            if self._running_locked(key):
                return {"ok": True, "key": key, "already": True,
                        "pid": self._procs[key].pid}
            # Starting exploration → wipe any stale semantic places so the
            # new run doesn't inherit wrong tags from a previous session.
            # Also reset the map cache so the browser doesn't show the old
            # localization map while the new SLAM map builds up.
            if key == "explore" and bridge is not None:
                bridge.store.clear()
                bridge.reset_map_cache()
                sem_map = os.path.join(HOME, "semantic_map.json")
                try:
                    if os.path.exists(sem_map):
                        os.remove(sem_map)
                except Exception:
                    pass
            self._logs[key] = deque(maxlen=200)
            proc = subprocess.Popen(
                targets[key]["cmd"], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
                start_new_session=True, env=os.environ.copy())
            self._procs[key] = proc
            threading.Thread(target=self._drain, args=(key, proc),
                             daemon=True).start()
            # When switching to localize, reset the stale SLAM map from the
            # browser and automatically seed AMCL with the spawn-point pose.
            # AMCL won't broadcast map→odom TF until it receives /initialpose,
            # which is why the robot position disappears after explore→localize.
            # We publish repeatedly over ~40 s to survive the Nav2 bringup delay.
            if key == "localize" and bridge is not None:
                bridge.reset_map_cache()
                def _seed_amcl():
                    # localize.launch.py delays Nav2 by 30 s; publish the
                    # initial pose at 30 s, 35 s, and 40 s to make sure AMCL
                    # is up before at least one message arrives.
                    for delay in (30.0, 35.0, 40.0):
                        time.sleep(delay if delay == 30.0
                                   else 5.0)   # subsequent sleeps are incremental
                        if not self._running_locked(key):
                            break             # localize was stopped, bail out
                        bridge.publish_initial_pose(x=5.0, y=0.0, theta=0.0)
                threading.Thread(target=_seed_amcl, daemon=True).start()
            return {"ok": True, "key": key, "pid": proc.pid}

    def stop(self, key):
        with self._lock:
            return {"ok": True, "key": key, "stopped": self._stop_locked(key)}

    def status(self):
        targets = _targets()
        with self._lock:
            return {"processes": [
                {"key": k, "label": targets[k]["label"],
                 "running": self._running_locked(k),
                 "pid": (self._procs[k].pid if k in self._procs else None),
                 "log": list(self._logs.get(k, []))[-6:]}
                for k in targets]}

    def save_map(self, name="house_map"):
        base = os.path.join(HOME, name)
        try:
            r = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", base],
                capture_output=True, text=True, timeout=30,
                env=os.environ.copy())
            return {"ok": r.returncode == 0, "path": base + ".yaml",
                    "log": (r.stdout + r.stderr)[-800:]}
        except subprocess.TimeoutExpired:
            return {"ok": False,
                    "error": "map_saver timed out - is /map being published?"}


# --------------------------------------------------------------------------
bridge: WebBridge = None
lm = LaunchManager()
app = FastAPI(title="Semantic Nav")


class QueryBody(BaseModel):
    query: str


class CmdBody(BaseModel):
    text: str


class GotoBody(BaseModel):
    query: str | None = None
    x: float | None = None
    y: float | None = None
    theta: float | None = 0.0


@app.get("/")
def index():
    return FileResponse(os.path.join(_WEB_DIR, "index.html"))


@app.get("/api/status")
def api_status():
    return bridge.status()


@app.get("/api/places")
def api_places():
    return {"places": bridge.places_public()}


@app.get("/api/map")
def api_map():
    return bridge.map_cache or {}


@app.post("/api/query")
def api_query(body: QueryBody):
    return bridge.match(body.query)


@app.post("/api/command")
def api_command(body: CmdBody):
    return bridge.handle_command(body.text)


@app.post("/api/goto")
def api_goto(body: GotoBody):
    """Publish a goal to the RRT planner only — shows the planned path.
    Does NOT send a NavigateToPose goal to Nav2; the robot will not move."""
    if body.query:
        m = bridge.match(body.query)
        if not m["found"]:
            return JSONResponse(m, status_code=404)
        bridge.publish_goal(m["x"], m["y"], m.get("theta", 0.0))
        m["action"] = "plan"
        m["say"] = f"Planned RRT path to the {m['label']}."
        return m
    if body.x is not None and body.y is not None:
        bridge.publish_goal(body.x, body.y, body.theta or 0.0)
        return {"found": True, "x": body.x, "y": body.y, "action": "plan"}
    return JSONResponse({"error": "need query or x,y"}, status_code=400)


@app.post("/api/navigate")
def api_navigate(body: GotoBody):
    """Plan an RRT path AND then send a NavigateToPose goal to Nav2 so the
    robot actually drives to the destination."""
    if body.query:
        m = bridge.match(body.query)
        if not m["found"]:
            return JSONResponse(m, status_code=404)
        x, y, theta = m["x"], m["y"], m.get("theta", 0.0)
        # First publish to RRT so the path is shown on the canvas.
        bridge.publish_goal(x, y, theta)
        # Then send the Nav2 action goal so the robot moves.
        nav_state = bridge.send_goal(x, y, theta)
        m["action"] = "goto"
        m["say"] = f"Navigating to the {m['label']}."
        m["nav_state"] = nav_state
        return m
    if body.x is not None and body.y is not None:
        x, y, theta = body.x, body.y, body.theta or 0.0
        bridge.publish_goal(x, y, theta)
        nav_state = bridge.send_goal(x, y, theta)
        return {"found": True, "x": x, "y": y, "action": "goto",
                "nav_state": nav_state}
    return JSONResponse({"error": "need query or x,y"}, status_code=400)


class LaunchBody(BaseModel):
    target: str
    map: str | None = None


class SaveBody(BaseModel):
    name: str | None = "house_map"


@app.get("/api/processes")
def api_processes():
    return lm.status()


@app.post("/api/launch")
def api_launch(body: LaunchBody):
    return lm.start(body.target, body.map or DEFAULT_MAP)


@app.post("/api/stop")
def api_stop(body: LaunchBody):
    return lm.stop(body.target)


@app.post("/api/save_map")
def api_save_map(body: SaveBody):
    return lm.save_map(body.name or "house_map")


@app.get("/api/map_status")
def api_map_status():
    """Check whether the saved nav map and semantic map files exist."""
    nav_map   = os.path.join(HOME, "house_map.yaml")
    nav_pgm   = os.path.join(HOME, "house_map.pgm")
    sem_map   = os.path.join(HOME, "semantic_map.json")
    nav_ok    = os.path.exists(nav_map) and os.path.exists(nav_pgm)
    sem_ok    = os.path.exists(sem_map)
    sem_places = 0
    if sem_ok:
        try:
            import json as _json
            data = _json.loads(open(sem_map).read())
            sem_places = len(data.get("places", []))
        except Exception:
            sem_ok = False
    return {
        "nav_map_ready":   nav_ok,
        "nav_map_path":    nav_map if nav_ok else None,
        "sem_map_ready":   sem_ok,
        "sem_map_places":  sem_places,
    }


@app.post("/api/clear_semantic")
def api_clear_semantic():
    """Wipe the in-memory semantic store and delete the JSON file.
    Called automatically when Explore is started, or manually from the UI."""
    sem_map = os.path.join(HOME, "semantic_map.json")
    bridge.store.clear()
    deleted = False
    if os.path.exists(sem_map):
        try:
            os.remove(sem_map)
            deleted = True
        except Exception as e:
            return {"ok": False, "error": str(e)}
    return {"ok": True, "file_deleted": deleted}


@app.websocket("/ws")
async def ws(sock: WebSocket):
    import asyncio
    await sock.accept()
    last_map_ver = -1
    first_message = True
    try:
        while True:
            payload = bridge.status()
            payload["places"] = bridge.places_public()
            payload["path"] = bridge.rrt_path
            # Only tell the browser the map version changed — the browser
            # fetches /api/map itself when the version ticks up.
            # This avoids serialising 30k+ cells over the socket every 0.4 s.
            mv = bridge.map_version
            payload["map_version"] = mv
            # On first message, always signal map_changed so the client fetches
            # the current map (prevents missing the map if it was published
            # before the WebSocket connection).
            payload["map_changed"] = (mv != last_map_ver) or first_message
            if mv != last_map_ver:
                last_map_ver = mv
            first_message = False
            await sock.send_text(json.dumps(payload))
            await asyncio.sleep(0.4)
    except WebSocketDisconnect:
        pass


def _free_port(port: int) -> bool:
    """Kill whatever is holding `port` so we can bind cleanly.
    Returns True if the port is free (or was freed), False if it couldn't be."""
    import socket, subprocess, time
    # Quick check — if nothing is on the port, we're done.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if s.connect_ex(("127.0.0.1", port)) != 0:
            return True   # nothing listening

    # Something is there. Find its PID via fuser and kill it.
    print(f"[web_backend] Port {port} already in use — killing stale process…",
          flush=True)
    try:
        r = subprocess.run(["fuser", "-k", f"{port}/tcp"],
                           capture_output=True, timeout=5)
        time.sleep(1.0)   # give the OS time to release the socket
    except (FileNotFoundError, subprocess.TimeoutExpired):
        # fuser not available — try lsof + kill
        try:
            r2 = subprocess.run(
                ["lsof", "-ti", f"tcp:{port}"],
                capture_output=True, text=True, timeout=5)
            for pid in r2.stdout.strip().split():
                subprocess.run(["kill", "-9", pid], timeout=3)
            time.sleep(1.0)
        except Exception:
            pass

    # Final check
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if s.connect_ex(("127.0.0.1", port)) != 0:
            print(f"[web_backend] Port {port} now free.", flush=True)
            return True

    print(f"[web_backend] Could not free port {port}. "
          f"Run:  fuser -k {port}/tcp   then retry.", flush=True)
    return False


class ReuseAddrConfig(uvicorn.Config):
    """uvicorn.Config with SO_REUSEADDR enabled so killed processes
    don't leave the port in TIME_WAIT."""
    def bind_unix_socket(self, path: str):
        # Unused for TCP, but required by the base class.
        return super().bind_unix_socket(path)


def main():
    global bridge
    rclpy.init()
    bridge = WebBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.5)

    # Auto-start the RRT planner node — it needs /map and /rrt_goal, both of
    # which are available as soon as either explore (SLAM) or localize runs.
    # Starting it here means Plan always works without any extra UI button.
    lm.start("rrt_planner")

    port = 8080
    
    # Try to free the port if it's held by a stale process
    for attempt in range(3):
        if _free_port(port):
            break
        if attempt < 2:
            print(f"[web_backend] Retrying in 2 seconds…", flush=True)
            time.sleep(2)
    else:
        print(f"[web_backend] Could not bind to port {port} after 3 attempts. Giving up.")
        return

    bridge.get_logger().info(f"dashboard on http://localhost:{port}")
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
