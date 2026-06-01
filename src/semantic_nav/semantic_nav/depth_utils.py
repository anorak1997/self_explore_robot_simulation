#!/usr/bin/env python3
"""
Depth fusion helpers.

The RGB classifier says *what* the robot is looking at. Depth (a depth
Image or a PointCloud2) says *how far* it is, which lets us:
  * reject frames where the nearest surface is too far (nothing really in
    view -> don't tag empty space), and
  * place the semantic tag at the observed surface, projected into the map
    frame, instead of at the robot's own position.

Both inputs are optional; if no depth is available the tagger falls back to
tagging at the robot pose. Stock TurtleBot3 waffle_pi has RGB only - add the
Intel RealSense model for /…/points and /…/depth/image_raw.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np


def center_depth_from_image(msg, patch=11) -> Optional[float]:
    """Median valid depth (metres) in a central patch of a depth Image.

    Handles 32FC1 (metres) and 16UC1 (millimetres)."""
    h, w = msg.height, msg.width
    if msg.encoding in ("32FC1", "32fc1"):
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(h, w)
        scale = 1.0
    elif msg.encoding in ("16UC1", "16uc1"):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(h, w)
        scale = 0.001
    else:
        return None

    cy, cx = h // 2, w // 2
    p = patch // 2
    win = arr[max(0, cy - p):cy + p + 1, max(0, cx - p):cx + p + 1].astype(float)
    win = win * scale
    valid = win[(win > 0.05) & np.isfinite(win)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def center_depth_from_cloud(msg, fwd_max=8.0) -> Optional[float]:
    """Nearest forward range (metres) from a PointCloud2.

    Uses points roughly ahead of the sensor (small |y|, |z|), takes a low
    percentile of x as the surface range."""
    try:
        from sensor_msgs_py import point_cloud2
    except Exception:
        return None
    xs = []
    for px, py, pz in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True):
        if 0.05 < px < fwd_max and abs(py) < 0.4 and abs(pz) < 0.4:
            xs.append(px)
    if not xs:
        return None
    return float(np.percentile(xs, 10))


def project_forward(robot_x, robot_y, robot_theta, distance,
                    cam_offset=0.10):
    """Point `distance` metres ahead of the robot, in the map frame."""
    r = cam_offset + distance
    return (robot_x + r * math.cos(robot_theta),
            robot_y + r * math.sin(robot_theta))
