#!/usr/bin/env python3
"""
Text embedding backend.

The whole point of Section 2 is that queries are NOT hard-coded. We
embed both the stored captions and the user's free-form query into the
same vector space and match by cosine similarity, so "where is the
toilet", "find the bathroom" and "where can I wash my hands" all land on
the same place without any keyword logic.

Two backends, selected automatically:

1. SentenceTransformerEmbedder - real open-vocabulary sentence
   embeddings (all-MiniLM-L6-v2). This is the recommended path and is
   the same idea as CLIP's TEXT encoder. Install with:
       pip install sentence-transformers

2. ConceptEmbedder - a transparent, zero-dependency fallback used when
   sentence-transformers is not installed. It maps words onto a small
   set of place concepts (with synonyms) so the demo still resolves
   synonyms reasonably. Good enough to demonstrate the architecture;
   swap in backend #1 for production-quality matching.
"""

from __future__ import annotations

import math
import re
from typing import List


def _normalize(vec: List[float]) -> List[float]:
    n = math.sqrt(sum(v * v for v in vec))
    if n == 0:
        return vec
    return [v / n for v in vec]


class SentenceTransformerEmbedder:
    name = "sentence-transformers/all-MiniLM-L6-v2"

    def __init__(self):
        from sentence_transformers import SentenceTransformer  # noqa
        self._model = SentenceTransformer("all-MiniLM-L6-v2")

    def embed(self, text: str) -> List[float]:
        vec = self._model.encode(text, normalize_embeddings=True)
        return [float(v) for v in vec]


class ConceptEmbedder:
    """Bag-of-concepts embedding with a small synonym table.

    Each dimension is a "place concept". A piece of text scores on a
    concept if it contains any of that concept's trigger words. The
    resulting vector is L2-normalized so cosine similarity behaves.
    """

    name = "concept-fallback"

    CONCEPTS = {
        "bathroom": ["bathroom", "toilet", "washroom", "restroom", "lavatory",
                     "sink", "wash", "hands", "pee", "loo", "wc"],
        "kitchen": ["kitchen", "pantry", "fridge", "stove", "oven", "cook",
                    "food", "eat", "coffee", "microwave", "snack"],
        "meeting": ["meeting", "conference", "boardroom", "presentation",
                    "projector", "whiteboard", "discuss", "standup"],
        "bedroom": ["bedroom", "bed", "sleep", "nap", "rest", "pillow"],
        "living": ["living", "lounge", "sofa", "couch", "tv", "relax",
                   "sitting"],
        "office": ["office", "desk", "workspace", "computer", "work",
                   "study"],
        "entrance": ["entrance", "entry", "door", "foyer", "lobby",
                     "reception", "hallway", "corridor", "hall",
                     "exit", "out", "outside", "leave"],
        "storage": ["storage", "closet", "store", "supplies", "cupboard"],
    }

    def __init__(self):
        self._keys = list(self.CONCEPTS.keys())

    def embed(self, text: str) -> List[float]:
        tokens = set(re.findall(r"[a-z]+", text.lower()))
        vec = []
        for k in self._keys:
            triggers = set(self.CONCEPTS[k])
            vec.append(float(len(tokens & triggers)))
        if sum(vec) == 0:
            # Unknown text -> zero vector matches nothing (cosine = 0),
            # so the query falls below threshold instead of false-matching.
            return [0.0] * len(self._keys)
        return _normalize(vec)


def get_embedder(verbose: bool = True):
    """Return the best available embedder, preferring the real model.

    Set env SEMANTIC_FORCE_FALLBACK=1 to always use the concept fallback.
    """
    import os
    if os.environ.get("SEMANTIC_FORCE_FALLBACK") == "1":
        return ConceptEmbedder()
    try:
        return SentenceTransformerEmbedder()
    except Exception as e:
        if verbose:
            import sys
            print(f"[embedding] sentence-transformers unavailable "
                  f"({type(e).__name__}: {e}); using concept fallback.",
                  file=sys.stderr)
        return ConceptEmbedder()
