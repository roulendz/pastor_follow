from __future__ import annotations
import time
from collections import deque
from .types import PerformanceStats


class PerformanceMonitor:
    """Tracks timing and provides quick metrics for debugging and tuning."""

    def __init__(self, history_len: int = 200):
        self.detect_ms = deque(maxlen=history_len)
        self.smooth_ms = deque(maxlen=history_len)
        self.coords_ms = deque(maxlen=history_len)
        self.control_ms = deque(maxlen=history_len)
        self.total_ms = deque(maxlen=history_len)

    def record(self, stats: PerformanceStats):
        self.detect_ms.append(stats.t_detect_ms)
        self.smooth_ms.append(stats.t_smooth_ms)
        self.coords_ms.append(stats.t_coords_ms)
        self.control_ms.append(stats.t_control_ms)
        self.total_ms.append(stats.t_total_ms)

    def summary(self) -> dict:
        def avg(q):
            return float(sum(q) / len(q)) if q else 0.0
        return {
            "avg_detect_ms": avg(self.detect_ms),
            "avg_smooth_ms": avg(self.smooth_ms),
            "avg_coords_ms": avg(self.coords_ms),
            "avg_control_ms": avg(self.control_ms),
            "avg_total_ms": avg(self.total_ms),
        }