"""
Raw tracking package

Provides a minimal, SRP/DRY head-tracking flow that works directly on
raw detector outputs without calibration transforms or clamping.
"""

from .raw_control import RawMovementController, RawSessionLogger

__all__ = [
    "RawMovementController",
    "RawSessionLogger",
]