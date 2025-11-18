from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2

from pose_detection import PoseDetector
from .raw_control import RawMovementController, RawSessionLogger


@dataclass
class RawTrackingResult:
    bDetected: bool
    fXraw: Optional[float]
    fXcol: Optional[float]
    fErr: Optional[float]
    fDbnDeg: float
    fOutDeltaDeg: float
    bSent: bool
    sReason: str
    obFrame: Optional[object]


class RawHeadTrackingPipeline:
    """Minimal raw pipeline: detect → compute → send → draw.

    - Uses PoseDetector directly to get raw normalized X for the target.
    - No inversion, offset, clamping, or smoothing.
    - Draws a simple overlay for clear visual feedback.
    """

    def __init__(self, obDetector: PoseDetector, obMover: RawMovementController, logger: Optional[RawSessionLogger] = None):
        self.obDetector = obDetector
        self.obMover = obMover
        self.logger = logger or RawSessionLogger()

    def step(self, obFrame) -> RawTrackingResult:
        # Detect raw position
        try:
            arPersons, obAnnotated = self.obDetector.process_frame(obFrame)
        except Exception as e:
            self.logger.log_failure(f"detection_error:{e}")
            return RawTrackingResult(
                bDetected=False, fXraw=None, fXcol=None, fErr=None, fDbnDeg=self.obMover.fDeadbandDeg,
                fOutDeltaDeg=0.0, bSent=False, sReason="detection_error", obFrame=obFrame,
            )

        # Pick nose, center-mode (raw)
        try:
            tplPos = self.obDetector.get_target_position(arPersons, body_part="nose", mode="center")
        except Exception:
            tplPos = None

        if not tplPos:
            return RawTrackingResult(
                bDetected=False, fXraw=None, fXcol=None, fErr=None, fDbnDeg=self.obMover.fDeadbandDeg,
                fOutDeltaDeg=0.0, bSent=False, sReason="no_target", obFrame=obAnnotated,
            )

        fXraw = float(tplPos[0])
        # Collision-adjusted equals raw for now (no transforms). Hook for future sensors.
        fXcol = fXraw

        # Error and command
        fErr = 0.5 - fXraw
        fOutDeltaDeg = self.obMover.compute_out_delta(fXraw)
        out = self.obMover.send_if_needed(fOutDeltaDeg)

        # Per-frame logging required by spec
        self.logger.log_fields(fXraw=fXraw, fXcol=fXcol, fErr=fErr, fDbn=self.obMover.fDeadbandDeg, fOutDeltaDeg=fOutDeltaDeg)
        self.obMover.record_error(fErr)

        # Simple overlay
        try:
            obFrameDbg = obAnnotated if obAnnotated is not None else obFrame
            self._draw_overlay(obFrameDbg, fXraw, fOutDeltaDeg)
        except Exception:
            obFrameDbg = obAnnotated if obAnnotated is not None else obFrame

        return RawTrackingResult(
            bDetected=True,
            fXraw=fXraw,
            fXcol=fXcol,
            fErr=fErr,
            fDbnDeg=self.obMover.fDeadbandDeg,
            fOutDeltaDeg=fOutDeltaDeg,
            bSent=out.bSent,
            sReason=out.sReason,
            obFrame=obFrameDbg,
        )

    def _draw_overlay(self, obFrame, fXraw: float, fOutDeltaDeg: float):
        h, w = obFrame.shape[:2]
        nCenterPx = int(0.5 * w)
        cv2.line(obFrame, (nCenterPx, 0), (nCenterPx, h), (200, 200, 200), 1)
        # Raw position line
        nXpx = int(float(fXraw) * w)
        cv2.line(obFrame, (nXpx, 0), (nXpx, h), (0, 255, 255), 1)
        # Direction arrow (white)
        nLen = int(40 * (1 if fOutDeltaDeg >= 0 else -1))
        nY = int(0.12 * h)
        cv2.arrowedLine(obFrame, (nCenterPx, nY), (nCenterPx + nLen, nY), (255, 255, 255), 2, tipLength=0.2)
        # Text block
        sDbg = (
            f"Xraw:{fXraw:.3f} Err:{(0.5-fXraw):.3f} "
            f"DBn:{self.obMover.fDeadbandDeg:.3f} OutDelta:{fOutDeltaDeg:.2f}degrees"
        )
        cv2.putText(obFrame, sDbg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)