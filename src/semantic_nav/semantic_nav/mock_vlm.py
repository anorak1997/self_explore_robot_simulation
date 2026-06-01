#!/usr/bin/env python3
"""
Mock vision-language model.

In a REAL deployment, `caption_image()` would run a vision model
(CLIP image encoder, BLIP-2, LLaVA, or a hosted vision API such as
GPT-4V / Claude vision) on the live camera frame and return either a
caption or an image embedding.

For this assignment the camera content is mocked: instead of looking at
pixels, we look at WHERE the robot is. Each labeled region of the world
(loaded from config/regions.yaml) has a human-style caption. When the
robot stands inside a region, the mock returns that region's caption -
exactly as a real captioner would if it were looking at the room.

Swapping in a real model is a ~5 line change inside `caption_image()`:
just feed `image` to the model and return its output. The rest of the
pipeline (storage, clustering, query, navigation) is unchanged.
"""

from __future__ import annotations

import base64
import os
from typing import List, Optional

import yaml


# Flat tints per room type for the mock "snapshot" thumbnails.
_TINTS = {
    "bathroom": ("#1d9e75", "#0f6e56"),
    "kitchen": ("#e0a23b", "#854f0b"),
    "meeting room": ("#7f77dd", "#3c3489"),
    "office": ("#378add", "#0c447c"),
    "entrance": ("#d85a30", "#712b13"),
}


def _thumbnail(label: str) -> str:
    """Return a small SVG snapshot as a data URI.

    This stands in for a real camera frame. With a real camera you would
    instead JPEG-encode the live image:
        import cv2, base64
        from cv_bridge import CvBridge
        cv = CvBridge().imgmsg_to_cv2(image, "bgr8")
        jpg = cv2.imencode(".jpg", cv)[1].tobytes()
        return "data:image/jpeg;base64," + base64.b64encode(jpg).decode()
    """
    c1, c2 = _TINTS.get(label, ("#5f5e5a", "#2c2c2a"))
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="120" height="90">'
        f'<rect width="120" height="90" fill="{c2}"/>'
        f'<rect width="120" height="58" fill="{c1}"/>'
        f'<circle cx="60" cy="30" r="16" fill="#ffffff" opacity="0.85"/>'
        f'<text x="60" y="78" font-family="monospace" font-size="12" '
        f'fill="#ffffff" text-anchor="middle">{label}</text></svg>'
    )
    b64 = base64.b64encode(svg.encode()).decode()
    return "data:image/svg+xml;base64," + b64


class MockVLM:
    def __init__(self, regions_path: str):
        self.regions = self._load_regions(regions_path)

    @staticmethod
    def _load_regions(path: str) -> List[dict]:
        if not path or not os.path.exists(path):
            return []
        with open(path, "r") as fh:
            data = yaml.safe_load(fh) or {}
        return data.get("regions", [])

    def caption_image(self, x: float, y: float, image=None) -> Optional[dict]:
        """Return {'label', 'caption', 'confidence'} for the robot's pose.

        `image` is accepted (and ignored) so the call signature already
        matches a real captioner. Returns None when the robot is not in
        any known region, so we don't pollute the map with junk tags.
        """
        for r in self.regions:
            xmin, ymin, xmax, ymax = r["bbox"]
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return {
                    "label": r["label"],
                    "caption": r["caption"],
                    "confidence": float(r.get("confidence", 0.9)),
                    "image": _thumbnail(r["label"]),
                    # Tag at the region CENTRE, not the robot pose, so every
                    # observation of one room lands on the same point and
                    # merges into a single place (no duplicate dots).
                    "x": (xmin + xmax) / 2.0,
                    "y": (ymin + ymax) / 2.0,
                }
        return None
