from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
import time


@dataclass
class FaceDetectionResult:
    """Represents the detected face position in normalized coordinates."""
    x: float  # normalized [0,1]
    y: float  # normalized [0,1]
    confidence: float
    timestamp: float
    annotated_frame: Optional[object] = None  # OpenCV BGR frame (np.ndarray)


@dataclass
class MoveCommand:
    """Movement command outcome."""
    delta_deg: float
    sent: bool
    reason: str  # e.g., "inside_deadband", "slew_limited", "throttled", "error"
    timestamp: float = time.time()


@dataclass
class PerformanceStats:
    """Timing metrics for each stage of the pipeline."""
    t_detect_ms: float = 0.0
    t_smooth_ms: float = 0.0
    t_coords_ms: float = 0.0
    t_control_ms: float = 0.0
    t_total_ms: float = 0.0


@dataclass
class PipelineOutput:
    """Aggregated output from one pipeline step."""
    detected: bool
    detection: Optional[FaceDetectionResult]
    measured_x_raw: Optional[float]
    measured_x_cal: Optional[float]
    error_norm: Optional[float]
    deadband_active: bool
    move: MoveCommand
    perf: PerformanceStats
    debug: Dict[str, float]