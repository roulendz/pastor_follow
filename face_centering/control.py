from __future__ import annotations
from typing import Optional
import time
import numpy as np

from .types import MoveCommand
from .errors import MovementError


class MovementController:
    """Owns movement decisions and sending to Arduino with rate limiting.

    - Uses provided PID controller for output
    - Applies slew rate limiting and command deadband
    - Enforces min command interval
    - Sends via ArduinoController.move_by_delta
    """

    def __init__(self, pid_controller, arduino_controller, config):
        self.pid = pid_controller
        self.arduino = arduino_controller
        self.config = config
        self._last_send_time = 0.0

    def compute_delta(self, measured_x: float, dt: float) -> float:
        raw = float(self.pid.update(measured_x, float(dt)))
        # Determine units of PID output: if output_limits span is small (≤2),
        # treat raw as normalized and apply output_scale_deg. If span is large,
        # assume degrees and skip scaling to avoid double-scaling.
        try:
            limits = getattr(self.pid, "output_limits", (-1.0, 1.0))
        except Exception:
            limits = (-1.0, 1.0)
        try:
            output_scale = float(self.config.get('pid', 'output_scale_deg') or 1.0)
        except Exception:
            output_scale = 1.0
        try:
            span = abs(float(limits[1]) - float(limits[0]))
        except Exception:
            span = 2.0
        if span <= 2.0:
            scaled = raw * output_scale
        else:
            scaled = raw
        try:
            slew = float(self.config.get('arduino', 'output_slew_rate_deg_per_sec'))
        except Exception:
            slew = 25.0
        max_delta = max(0.0, slew * float(dt))
        return float(np.clip(scaled, -max_delta, max_delta))

    def maybe_send(self, delta_deg: float) -> MoveCommand:
        try:
            min_interval_ms = float(self.config.get('arduino', 'min_command_interval_ms'))
            cmd_deadband_deg = float(self.config.get('arduino', 'command_deadband_deg'))
        except Exception:
            min_interval_ms = 40.0
            cmd_deadband_deg = 0.2

        now = time.time()
        if abs(delta_deg) < cmd_deadband_deg:
            return MoveCommand(delta_deg=0.0, sent=False, reason="below_cmd_deadband")
        if (now - self._last_send_time) * 1000.0 < min_interval_ms:
            return MoveCommand(delta_deg=delta_deg, sent=False, reason="throttled")

        if not getattr(self.arduino, "connected", False):
            return MoveCommand(delta_deg=0.0, sent=False, reason="arduino_disconnected")

        try:
            ok = self.arduino.move_by_delta(float(delta_deg))
        except Exception as e:
            raise MovementError(f"arduino send failed: {e}")
        if ok:
            self._last_send_time = now
            return MoveCommand(delta_deg=float(delta_deg), sent=True, reason="sent")
        return MoveCommand(delta_deg=float(delta_deg), sent=False, reason="controller_rejected")