"""
Raw tracking package

Provides a minimal, SRP/DRY head-tracking flow that works directly on
raw detector outputs without calibration transforms or clamping.
"""

from .raw_control import RawMovementController, RawSessionLogger
from .raw_pipeline import RawHeadTrackingPipeline, RawTrackingResult

__all__ = [
    "RawMovementController",
    "RawSessionLogger",
    "RawHeadTrackingPipeline",
    "RawTrackingResult",
]