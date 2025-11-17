from __future__ import annotations
from typing import Optional, Tuple
import time


class SmoothingPipeline:
    """Wraps the existing PositionSmoother and exposes a simple API.

    Keeps SRP by isolating the smoothing concerns and runtime configuration.
    """

    def __init__(self, smoother, config):
        self.smoother = smoother
        self.config = config

    def update(self, xy: Tuple[float, float]) -> Tuple[float, float]:
        enabled = bool(self.config.get('tracking', 'smoothing_enabled'))
        if not enabled:
            return float(xy[0]), float(xy[1])
        s = self.smoother.update((float(xy[0]), float(xy[1])))
        return (float(s[0]), float(s[1])) if s is not None else (float(xy[0]), float(xy[1]))