from __future__ import annotations
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Callable


@dataclass
class RawMoveOutcome:
    fDeltaDeg: float
    bSent: bool
    sReason: str
    fTimestamp: float


class RawSessionLogger:
    """File-only session logger tailored to required fields.

    Logs compact, parseable lines to `last_session.log` while avoiding UI coupling.
    """

    def __init__(self, sLogPath: str = "f:\\Documents\\Arduino\\pastor_follow_v2\\last_session.log"):
        self.sLogPath = sLogPath
        try:
            # Open in append mode so previous sessions are preserved
            self._fh = open(self.sLogPath, "a", encoding="utf-8")
        except Exception:
            self._fh = None

    def log_fields(self, fXraw: float, fXcol: float, fErr: float, fDbn: float, fOutDeltaDeg: float):
        sLine = (
            f"Xraw:{fXraw:.3f} Xcol:{fXcol:.3f} Err:{fErr:.3f} "
            f"DBn:{fDbn:.3f} OutDelta:{fOutDeltaDeg:.2f}degrees"
        )
        self._write(sLine)

    def log_info(self, sMsg: str):
        self._write(sMsg)

    def log_failure(self, sMsg: str):
        self._write(f"FAIL: {sMsg}")

    def _write(self, sLine: str):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {sLine}"
        try:
            if self._fh:
                self._fh.write(line + "\n")
                self._fh.flush()
        except Exception:
            pass

    def close(self):
        try:
            if self._fh:
                self._fh.close()
        except Exception:
            pass


class RawMovementController:
    """SRP controller: compute OutDelta and send immediately.

    - Works directly on raw normalized X (`fXraw`), no calibration/inversion/clamps.
    - Applies only: center error, deadband in degrees, and degree scaling.
    - Sends through ArduinoController with immediate intent.
    - Captures detailed failures and long-term centering issues.
    """

    def __init__(
        self,
        obArduino,
        fScaleDegPerUnit: float = 60.0,
        fDeadbandDeg: float = 0.0,
        logger: Optional[RawSessionLogger] = None,
        fnOnSend: Optional[Callable[[float], None]] = None,
    ):
        self.obArduino = obArduino
        self.fScaleDegPerUnit = float(fScaleDegPerUnit)
        self.fDeadbandDeg = float(fDeadbandDeg)
        self.logger = logger or RawSessionLogger()
        self.fnOnSend = fnOnSend

        # Diagnostics
        self._dqErrAbs = deque(maxlen=300)  # ~10s at 30 FPS
        self._lastSend = 0.0
        self._lastDelta = 0.0
        self._lastFailureCheck = 0.0

    def compute_out_delta(self, fXraw: float) -> float:
        fErr = 0.5 - float(fXraw)
        return fErr * self.fScaleDegPerUnit

    def send_if_needed(self, fOutDeltaDeg: float) -> RawMoveOutcome:
        fNow = time.time()
        bWithinDbn = abs(float(fOutDeltaDeg)) < self.fDeadbandDeg
        if bWithinDbn:
            return RawMoveOutcome(fDeltaDeg=0.0, bSent=False, sReason="below_deadband", fTimestamp=fNow)

        bOk = False
        sReason = "sent"
        try:
            # Intend immediate motion; let Arduino enforce any physical limits.
            bOk = bool(self.obArduino.move_by_delta(float(fOutDeltaDeg)))
            if not bOk:
                sReason = "arduino_rejected"
        except Exception as e:
            sReason = f"arduino_error:{e}"
            bOk = False

        self._lastSend = fNow if bOk else self._lastSend
        self._lastDelta = float(fOutDeltaDeg)
        if self.fnOnSend and bOk:
            try:
                self.fnOnSend(float(fOutDeltaDeg))
            except Exception:
                pass

        # Immediate failure diagnostics
        # Log explicit "not moving" cases
        bMoving = bool(getattr(self.obArduino, 'is_moving', False))
        if not bOk or (not bMoving and abs(float(fOutDeltaDeg)) > self.fDeadbandDeg):
            self.logger.log_failure(
                f"OutDelta {float(fOutDeltaDeg):.2f} degrees and motor not moving"
            )

        return RawMoveOutcome(fDeltaDeg=float(fOutDeltaDeg), bSent=bOk, sReason=sReason, fTimestamp=fNow)

    def record_error(self, fErr: float):
        self._dqErrAbs.append(abs(float(fErr)))
        self._maybe_report_long_term_failure()

    def _maybe_report_long_term_failure(self):
        fNow = time.time()
        if (fNow - self._lastFailureCheck) < 2.0:
            return
        self._lastFailureCheck = fNow
        if not self._dqErrAbs:
            return
        fAvgErr = sum(self._dqErrAbs) / len(self._dqErrAbs)
        # If average normalized error stays above 0.04 (~8% of frame width) for ~10s
        if fAvgErr > 0.04 and len(self._dqErrAbs) == self._dqErrAbs.maxlen:
            self.logger.log_failure(
                f"Centering failure: avg error {fAvgErr:.3f} over ~{len(self._dqErrAbs)/30:.1f}s"
            )