#!/usr/bin/env python3
"""
RRT / RRT* on a 2D occupancy grid.

Pure Python + numpy, no ROS dependency, so it can be unit-tested on its
own. The ROS node (rrt_planner_node.py) is a thin wrapper that feeds it a
nav_msgs/OccupancyGrid and publishes the result as a nav_msgs/Path.

Handles the three things the brief calls out explicitly:
  * grid resolution + origin  -> world<->grid conversion
  * obstacle inflation        -> dilate occupied cells by a radius
  * unknown cells             -> treated as obstacles by default (safe)
"""

from __future__ import annotations

import math
import random
from typing import List, Optional, Tuple

import numpy as np

Point = Tuple[float, float]


class RRTPlanner:
    def __init__(self, occupancy: np.ndarray, resolution: float,
                 origin_x: float, origin_y: float,
                 inflation_radius: float = 0.20,
                 occ_threshold: int = 50,
                 treat_unknown_as_obstacle: bool = True,
                 step_size: float = 0.30,
                 goal_bias: float = 0.10,
                 max_iters: int = 5000,
                 goal_tolerance: float = 0.25,
                 rrt_star: bool = False,
                 rewire_radius: float = 0.60,
                 seed: Optional[int] = None):
        """`occupancy` is a 2D int array shaped (height, width), row-major,
        with values -1 (unknown), 0 (free) .. 100 (occupied)."""
        self.res = resolution
        self.ox = origin_x
        self.oy = origin_y
        self.step = step_size
        self.goal_bias = goal_bias
        self.max_iters = max_iters
        self.goal_tol = goal_tolerance
        self.star = rrt_star
        self.rewire_radius = rewire_radius
        if seed is not None:
            random.seed(seed)

        h, w = occupancy.shape
        self.h, self.w = h, w

        blocked = occupancy >= occ_threshold
        if treat_unknown_as_obstacle:
            blocked |= (occupancy < 0)
        self.blocked = self._inflate(blocked, int(round(inflation_radius / resolution)))

    # ---- grid helpers ----------------------------------------------------
    @staticmethod
    def _inflate(grid: np.ndarray, cells: int) -> np.ndarray:
        if cells <= 0:
            return grid
        h, w = grid.shape
        padded = np.pad(grid, cells)            # zero-filled border
        out = np.zeros_like(grid)
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                if dx * dx + dy * dy > cells * cells:   # circular kernel
                    continue
                out |= padded[cells + dy: cells + dy + h,
                              cells + dx: cells + dx + w]
        return out

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ix = int((x - self.ox) / self.res)
        iy = int((y - self.oy) / self.res)
        return ix, iy

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.w and 0 <= iy < self.h

    def point_free(self, x: float, y: float) -> bool:
        ix, iy = self.world_to_grid(x, y)
        if not self.in_bounds(ix, iy):
            return False
        return not self.blocked[iy, ix]

    def segment_free(self, a: Point, b: Point) -> bool:
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d / (self.res * 0.5)))
        for i in range(n + 1):
            t = i / n
            if not self.point_free(a[0] + (b[0] - a[0]) * t,
                                   a[1] + (b[1] - a[1]) * t):
                return False
        return True

    # ---- sampling bounds -------------------------------------------------
    def _world_bounds(self):
        return (self.ox, self.oy,
                self.ox + self.w * self.res, self.oy + self.h * self.res)

    def _sample(self, goal: Point) -> Point:
        if random.random() < self.goal_bias:
            return goal
        minx, miny, maxx, maxy = self._world_bounds()
        return (random.uniform(minx, maxx), random.uniform(miny, maxy))

    @staticmethod
    def _nearest(nodes: List[Point], p: Point) -> int:
        best, bd = 0, float("inf")
        for i, n in enumerate(nodes):
            d = (n[0] - p[0]) ** 2 + (n[1] - p[1]) ** 2
            if d < bd:
                best, bd = i, d
        return best

    def _steer(self, a: Point, b: Point) -> Point:
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d <= self.step:
            return b
        return (a[0] + (b[0] - a[0]) / d * self.step,
                a[1] + (b[1] - a[1]) / d * self.step)

    # ---- main ------------------------------------------------------------
    def plan(self, start: Point, goal: Point):
        """Return dict: {found, path, edges, iters, length}."""
        if not self.point_free(*start):
            return {"found": False, "reason": "start in collision",
                    "path": [], "edges": [], "iters": 0, "length": 0.0}
        if not self.point_free(*goal):
            return {"found": False, "reason": "goal in collision",
                    "path": [], "edges": [], "iters": 0, "length": 0.0}

        nodes: List[Point] = [start]
        parent: List[int] = [-1]
        cost: List[float] = [0.0]
        edges: List[Tuple[Point, Point]] = []
        goal_idx = -1

        for it in range(self.max_iters):
            s = self._sample(goal)
            ni = self._nearest(nodes, s)
            new = self._steer(nodes[ni], s)
            if not self.segment_free(nodes[ni], new):
                continue

            best_parent = ni
            best_cost = cost[ni] + math.dist(nodes[ni], new)
            if self.star:
                for j, nd in enumerate(nodes):
                    if math.dist(nd, new) <= self.rewire_radius and \
                            self.segment_free(nd, new):
                        c = cost[j] + math.dist(nd, new)
                        if c < best_cost:
                            best_parent, best_cost = j, c

            nodes.append(new)
            parent.append(best_parent)
            cost.append(best_cost)
            edges.append((nodes[best_parent], new))
            new_idx = len(nodes) - 1

            if self.star:
                for j, nd in enumerate(nodes[:-1]):
                    if math.dist(nd, new) <= self.rewire_radius and \
                            self.segment_free(new, nd):
                        c = best_cost + math.dist(new, nd)
                        if c < cost[j]:
                            parent[j] = new_idx
                            cost[j] = c

            if math.dist(new, goal) <= self.goal_tol and \
                    self.segment_free(new, goal):
                nodes.append(goal)
                parent.append(new_idx)
                cost.append(best_cost + math.dist(new, goal))
                goal_idx = len(nodes) - 1
                if not self.star:
                    break

        if goal_idx < 0:
            return {"found": False, "reason": "max iters",
                    "path": [], "edges": edges, "iters": self.max_iters,
                    "length": 0.0}

        path = []
        i = goal_idx
        while i != -1:
            path.append(nodes[i])
            i = parent[i]
        path.reverse()
        length = sum(math.dist(path[k], path[k + 1])
                     for k in range(len(path) - 1))
        return {"found": True, "path": path, "edges": edges,
                "iters": it + 1, "length": length}
