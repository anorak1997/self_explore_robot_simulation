#!/usr/bin/env python3
"""
The robot's semantic memory.

Observations stream in as (caption, pose, embedding). We don't keep every
frame - observations that are close in space and share a label are merged
into a single PLACE with a running-average pose and a running-average
embedding. Each place is one entry the user can later navigate to.

The store serializes to plain JSON so it survives restarts and can be
shared between the map node, the query node and the web backend.
"""

from __future__ import annotations

import json
import math
import threading
from typing import List, Optional


def cosine(a: List[float], b: List[float]) -> float:
    if not a or not b or len(a) != len(b):
        return 0.0
    dot = sum(x * y for x, y in zip(a, b))
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(y * y for y in b))
    if na == 0 or nb == 0:
        return 0.0
    return dot / (na * nb)


class SemanticStore:
    def __init__(self, merge_distance: float = 1.0):
        self.merge_distance = merge_distance
        self._places: List[dict] = []
        self._next_id = 0
        self._lock = threading.Lock()

    # ---- writing ---------------------------------------------------------
    def add_observation(self, label, caption, x, y, theta,
                        embedding, confidence, image="") -> dict:
        with self._lock:
            match = self._nearest_same_label(label, x, y)
            if match is not None:
                n = match["count"]
                match["x"] = (match["x"] * n + x) / (n + 1)
                match["y"] = (match["y"] * n + y) / (n + 1)
                match["theta"] = theta
                match["embedding"] = [
                    (e * n + f) / (n + 1)
                    for e, f in zip(match["embedding"], embedding)
                ]
                match["confidence"] = max(match["confidence"], confidence)
                match["count"] = n + 1
                if image and not match.get("image"):
                    match["image"] = image
                return match

            place = {
                "id": self._next_id,
                "label": label,
                "caption": caption,
                "x": float(x),
                "y": float(y),
                "theta": float(theta),
                "embedding": list(embedding),
                "confidence": float(confidence),
                "image": image,
                "count": 1,
            }
            self._next_id += 1
            self._places.append(place)
            return place

    def _nearest_same_label(self, label, x, y) -> Optional[dict]:
        best, best_d = None, self.merge_distance
        for p in self._places:
            if p["label"] != label:
                continue
            d = math.hypot(p["x"] - x, p["y"] - y)
            if d < best_d:
                best, best_d = p, d
        return best

    def clear(self):
        """Wipe all places (called when switching to a fresh exploration run)."""
        with self._lock:
            self._places = []
            self._next_id = 0

    # ---- reading ---------------------------------------------------------
    def list_places(self) -> List[dict]:
        with self._lock:
            return [dict(p) for p in self._places]

    def query(self, query_embedding: List[float]):
        """Return (best_place, score) or (None, 0.0)."""
        with self._lock:
            best, best_score = None, -1.0
            for p in self._places:
                s = cosine(query_embedding, p["embedding"])
                if s > best_score:
                    best, best_score = p, s
            if best is None:
                return None, 0.0
            return dict(best), float(best_score)

    # ---- persistence -----------------------------------------------------
    def to_json(self) -> str:
        with self._lock:
            return json.dumps({"places": self._places}, indent=2)

    def load_json(self, text: str):
        data = json.loads(text)
        with self._lock:
            self._places = data.get("places", [])
            self._next_id = max((p["id"] for p in self._places), default=-1) + 1

    def save(self, path: str):
        with open(path, "w") as fh:
            fh.write(self.to_json())

    def load(self, path: str):
        with open(path, "r") as fh:
            self.load_json(fh.read())
