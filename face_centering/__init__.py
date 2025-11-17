"""
face_centering package
Modular face centering pipeline following SOLID and DRY.
"""

from .types import FaceDetectionResult, MoveCommand, PerformanceStats, PipelineOutput
from .errors import FaceDetectionError, CoordinateError, MovementError
from .logger import EventLogger
from .detection import MediaPipeFaceDetectorAdapter
from .coordinates import CoordinateMapper
from .smoothing import SmoothingPipeline
from .control import MovementController
from .monitor import PerformanceMonitor
from .pipeline import FaceCenteringPipeline

__all__ = [
    "FaceDetectionResult",
    "MoveCommand",
    "PerformanceStats",
    "PipelineOutput",
    "FaceDetectionError",
    "CoordinateError",
    "MovementError",
    "EventLogger",
    "MediaPipeFaceDetectorAdapter",
    "CoordinateMapper",
    "SmoothingPipeline",
    "MovementController",
    "PerformanceMonitor",
    "FaceCenteringPipeline",
]