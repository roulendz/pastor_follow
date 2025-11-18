from __future__ import annotations
import time
from typing import Optional


def _iso_ts_ms(t: Optional[float] = None) -> str:
    """Return timestamp with millisecond precision (CSV friendly)."""
    if t is None:
        t = time.time()
    base = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))
    ms = int((t - int(t)) * 1000)
    return f"{base}.{ms:03d}"


class SliderCSVLogger:
    """CSV logger for slider/motor telemetry with condition tracking.

    Columns:
    - timestamp
    - mode (normal|restricted)
    - slider_norm (-1..1)
    - motor_angle_deg
    - motor_speed_deg_s
    - slow_movement (0|1)
    - unidirectional (0|1)
    - slow_duration_ms
    - unidir_duration_ms
    """

    def __init__(
        self,
        log_path: str = "f:\\Documents\\Arduino\\pastor_follow_v2\\last_session.log",
        slow_threshold_deg_s: float = 1.0,
        engage_threshold_norm: float = 0.2,
        unidir_hold_ms: int = 250,
        write_header: bool = True,
    ):
        self.log_path = log_path
        self.slow_threshold = float(slow_threshold_deg_s)
        self.engage_threshold = float(engage_threshold_norm)
        self.unidir_hold_ms = int(unidir_hold_ms)

        self._fh = None
        try:
            self._fh = open(self.log_path, "a", encoding="utf-8")
            if write_header:
                self._write_header()
        except Exception:
            self._fh = None

        # Condition state
        self._slow_active = False
        self._slow_start = 0.0

        self._unidir_active = False
        self._unidir_start = 0.0
        self._mismatch_start = 0.0

    def _write_header(self):
        hdr = (
            "timestamp,mode,slider_norm,motor_angle_deg,motor_speed_deg_s,"
            "slow_movement,unidirectional,slow_duration_ms,unidir_duration_ms"
        )
        try:
            if self._fh:
                self._fh.write(hdr + "\n")
                self._fh.flush()
        except Exception:
            pass

    def log_sample(self, slider_norm: float, motor_angle_deg: float, motor_speed_deg_s: float):
        t = time.time()

        # Slow movement detection: desired engagement but measured speed below threshold
        if abs(slider_norm) >= self.engage_threshold and abs(motor_speed_deg_s) < self.slow_threshold:
            if not self._slow_active:
                self._slow_active = True
                self._slow_start = t
            slow_ms = int((t - self._slow_start) * 1000)
        else:
            self._slow_active = False
            slow_ms = 0

        # Unidirectional limitation detection:
        # If desired sign (from slider) differs from measured sign for a sustained period.
        desired_sign = 0
        if abs(slider_norm) >= self.engage_threshold:
            desired_sign = 1 if slider_norm > 0 else -1
        measured_sign = 1 if motor_speed_deg_s > 0 else (-1 if motor_speed_deg_s < 0 else 0)

        if desired_sign != 0:
            mismatch = (measured_sign != 0) and (measured_sign != desired_sign)
            if mismatch:
                if self._mismatch_start == 0.0:
                    self._mismatch_start = t
                hold_ms = int((t - self._mismatch_start) * 1000)
                if hold_ms >= self.unidir_hold_ms:
                    if not self._unidir_active:
                        self._unidir_active = True
                        self._unidir_start = self._mismatch_start
                    unidir_ms = int((t - self._unidir_start) * 1000)
                else:
                    self._unidir_active = False
                    unidir_ms = 0
            else:
                self._mismatch_start = 0.0
                self._unidir_active = False
                unidir_ms = 0
        else:
            self._mismatch_start = 0.0
            self._unidir_active = False
            unidir_ms = 0

        mode = "restricted" if (self._slow_active or self._unidir_active) else "normal"
        row = (
            f"{_iso_ts_ms(t)},{mode},{float(slider_norm):.3f},{float(motor_angle_deg):.3f},"
            f"{float(motor_speed_deg_s):.3f},{int(self._slow_active)},{int(self._unidir_active)},"
            f"{int(slow_ms)},{int(unidir_ms)}"
        )

        try:
            if self._fh:
                self._fh.write(row + "\n")
                # Flush to ensure <10ms latency visibility in monitors
                self._fh.flush()
        except Exception:
            pass

    def close(self):
        try:
            if self._fh:
                self._fh.close()
        except Exception:
            pass