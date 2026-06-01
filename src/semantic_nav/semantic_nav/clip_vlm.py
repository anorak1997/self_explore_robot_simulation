#!/usr/bin/env python3
"""
Real vision-language backend: open-vocabulary zero-shot room classification
with CLIP.

Why this keeps queries non-hard-coded: CLIP embeds the camera image and a
set of *text* candidate prompts ("a photo of a kitchen", ...) into the same
space and picks the closest. The candidate list lives in config/labels.yaml
and can be edited freely; there is no if/else on room names anywhere. This
is the image-side twin of the text matching done in embedding.py.

Runs on CPU (a few hundred ms per frame, fine for a 2 s tag period). Uses
HuggingFace transformers CLIP, which ships with sentence-transformers.

Image decoding is done manually from sensor_msgs/Image bytes so we don't
depend on cv_bridge (which has ABI quirks across distros).
"""

from __future__ import annotations

import base64
import io
from typing import List, Optional

import numpy as np


def decode_image_msg(msg) -> Optional["np.ndarray"]:
    """sensor_msgs/Image -> HxWx3 uint8 RGB array. Handles rgb8/bgr8/mono8."""
    enc = msg.encoding
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    h, w = msg.height, msg.width
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, w, 3)
        if enc == "bgr8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    if enc == "mono8":
        g = buf.reshape(h, w)
        return np.stack([g, g, g], axis=-1)
    if enc == "rgba8" or enc == "bgra8":
        img = buf.reshape(h, w, 4)[:, :, :3]
        if enc == "bgra8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    return None


def to_thumbnail(rgb: "np.ndarray", max_w: int = 160) -> str:
    """RGB array -> small JPEG data URI for the dashboard."""
    try:
        from PIL import Image
        im = Image.fromarray(rgb)
        if im.width > max_w:
            im = im.resize((max_w, int(im.height * max_w / im.width)))
        out = io.BytesIO()
        im.save(out, format="JPEG", quality=70)
        b64 = base64.b64encode(out.getvalue()).decode()
        return "data:image/jpeg;base64," + b64
    except Exception:
        return ""


class ClipVLM:
    """Zero-shot room classifier over a configurable label set."""

    def __init__(self, labels: List[dict], model_name="openai/clip-vit-base-patch32",
                 min_confidence: float = 0.30):
        from transformers import CLIPModel, CLIPProcessor
        import torch
        self.torch = torch
        self.device = "cpu"
        self.model = CLIPModel.from_pretrained(model_name).to(self.device).eval()
        self.proc = CLIPProcessor.from_pretrained(model_name)
        self.min_conf = min_confidence

        self.labels = labels                      # [{label, caption, prompt?}]
        self.prompts = [l.get("prompt", f"a photo of {l['label']}")
                        for l in labels]
        # precompute text features once
        with torch.no_grad():
            ti = self.proc(text=self.prompts, return_tensors="pt",
                           padding=True).to(self.device)
            tf = self.model.get_text_features(**ti)
            self.text_feats = tf / tf.norm(dim=-1, keepdim=True)

    def classify(self, rgb: "np.ndarray") -> Optional[dict]:
        """Return {label, caption, confidence, image} or None if uncertain."""
        from PIL import Image
        torch = self.torch
        pil = Image.fromarray(rgb)
        with torch.no_grad():
            ii = self.proc(images=pil, return_tensors="pt").to(self.device)
            f = self.model.get_image_features(**ii)
            f = f / f.norm(dim=-1, keepdim=True)
            sims = (f @ self.text_feats.T).squeeze(0)
            probs = sims.softmax(dim=-1)
            idx = int(probs.argmax())
            conf = float(probs[idx])

        if conf < self.min_conf:
            return None
        sel = self.labels[idx]
        return {
            "label": sel["label"],
            "caption": sel.get("caption", f"a view of {sel['label']}"),
            "confidence": conf,
            "image": to_thumbnail(rgb),
        }
