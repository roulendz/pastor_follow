from __future__ import annotations
from typing import Optional, Tuple
import time

from .types import FaceDetectionResult
from .errors import FaceDetectionError


class IFaceDetector:
    """Interface for face detection components."""

    def detect(self, frame) -> FaceDetectionResult:
        raise NotImplementedError


class MediaPipeFaceDetectorAdapter(IFaceDetector):
    """Adapter wrapping the existing PoseDetector to produce face positions.

    - Selects target body part per config (default: nose)
    - Applies tracking mode selection (center/largest/closest)
    """

    def __init__(self, pose_detector, config):
        self.pose_detector = pose_detector
        self.config = config

    def detect(self, frame) -> FaceDetectionResult:
        t0 = time.time()
        try:
            persons, annotated_frame = self.pose_detector.process_frame(frame)
        except Exception as e:
            raise FaceDetectionError(f"pose processing failed: {e}")

        if not persons:
            raise FaceDetectionError("no persons detected")

        try:
            body_part = self.config.get('tracking', 'target_body_part')
            mode = self.config.get('tracking', 'tracking_mode')
            target_pos = self.pose_detector.get_target_position(persons, body_part, mode)
            if not target_pos:
                raise FaceDetectionError("no target position returned")
            x = float(target_pos[0])
            y = float(target_pos[1])
        except Exception as e:
            raise FaceDetectionError(f"target selection failed: {e}")

        try:
            min_vis = float(self.config.get('pose_detection', 'min_visibility') or 0.0)
        except Exception:
            min_vis = 0.0
        # The adapter does not expose visibility directly; assume sufficient if returned
        confidence = max(0.0, min(1.0, min_vis))
        return FaceDetectionResult(x=x, y=y, confidence=confidence, timestamp=time.time(), annotated_frame=annotated_frame)