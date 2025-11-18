from __future__ import annotations
from typing import Optional
import time
import numpy as np

from .types import MoveCommand
from .errors import MovementError


class MovementController:
    """Owns movement decisions and sending to Arduino.

    Refactored for SRP/DRY:
    - compute_delta: PID + slew limiting (single responsibility)
    - helper methods: typed config access and independent gates
    - maybe_send: linear pipeline of simple checks, no nested logic
    """

    def __init__(self, pid_controller, arduino_controller, config):
        self.pid = pid_controller
        self.arduino = arduino_controller
        self.config = config
        self._last_send_time = 0.0
        # Track last non-zero command sign to prevent rapid ping-pong
        self._last_command_sign = 0
        # EMA smoothing state
        self._delta_ema = 0.0
        # Accumulate deltas between allowed send windows to avoid command spam
        self._pending_delta = 0.0

    # --- Config helpers (DRY) ---
    def _get_float(self, section: str, key: str, default: float) -> float:
        try:
            val = self.config.get(section, key)
            return default if val is None else float(val)
        except Exception:
            return default

    def _rules(self):
        return {
            'min_interval_ms': self._get_float('arduino', 'min_command_interval_ms', 40.0),
            'cmd_deadband_deg': self._get_float('arduino', 'command_deadband_deg', 0.2),
            'sign_flip_guard_deg': self._get_float('arduino', 'sign_flip_guard_deg', 0.0),
            'delta_alpha': self._get_float('arduino', 'cmd_delta_alpha', 0.5),
            'moving_hold_speed': self._get_float('arduino', 'moving_hold_speed_deg_per_sec', 0.0),
            'slew_rate_deg_s': self._get_float('arduino', 'output_slew_rate_deg_per_sec', 25.0),
            'output_scale_deg': self._get_float('pid', 'output_scale_deg', 1.0),
        }

    # --- Processing helpers ---
    @staticmethod
    def _sign(x: float) -> int:
        return 1 if x > 0 else (-1 if x < 0 else 0)

    def _smooth_delta(self, delta: float, alpha: float) -> float:
        sm = alpha * float(delta) + (1.0 - alpha) * float(self._delta_ema)
        self._delta_ema = float(sm)
        return sm

    def compute_delta(self, measured_x: float, dt: float) -> float:
        rules = self._rules()
        raw = float(self.pid.update(measured_x, float(dt)))
        limits = getattr(self.pid, "output_limits", (-1.0, 1.0))
        try:
            span = abs(float(limits[1]) - float(limits[0]))
        except Exception:
            span = 2.0
        scaled = raw * rules['output_scale_deg'] if span <= 2.0 else raw
        max_delta = max(0.0, rules['slew_rate_deg_s'] * float(dt))
        return float(np.clip(scaled, -max_delta, max_delta))

    def maybe_send(self, delta_deg: float) -> MoveCommand:
        rules = self._rules()
        now = time.time()

        # Smooth and accumulate pending correction
        d = self._smooth_delta(delta_deg, rules['delta_alpha'])
        self._pending_delta += float(d)
        p = float(self._pending_delta)

        # Gate on command deadband using the accumulated correction
        if abs(p) < rules['cmd_deadband_deg']:
            return MoveCommand(delta_deg=0.0, sent=False, reason="below_cmd_deadband")

        cur_sign = self._sign(p)
        if (
            rules['sign_flip_guard_deg'] > 0.0
            and self._last_command_sign != 0
            and cur_sign != 0
            and cur_sign != self._last_command_sign
            and abs(p) <= rules['sign_flip_guard_deg']
        ):
            return MoveCommand(delta_deg=0.0, sent=False, reason="sign_flip_guard")

        # Hold tiny corrections while motor is moving fast
        if rules['moving_hold_speed'] > 0.0:
            fb = getattr(self.arduino, 'last_feedback', None)
            speed = float(getattr(fb, 'current_speed', 0.0)) if fb is not None else 0.0
            if (
                bool(getattr(self.arduino, 'is_moving', False))
                and abs(speed) >= rules['moving_hold_speed']
                and abs(p) <= max(rules['cmd_deadband_deg'] * 2.0, rules['sign_flip_guard_deg'])
            ):
                return MoveCommand(delta_deg=0.0, sent=False, reason="moving_fast_hold")

        if (now - self._last_send_time) * 1000.0 < rules['min_interval_ms']:
            return MoveCommand(delta_deg=p, sent=False, reason="throttled")

        if not getattr(self.arduino, "connected", False):
            return MoveCommand(delta_deg=0.0, sent=False, reason="arduino_disconnected")

        try:
            ok = self.arduino.move_by_delta(p)
        except Exception as e:
            raise MovementError(f"arduino send failed: {e}")
        if ok:
            self._last_send_time = now
            self._last_command_sign = cur_sign if cur_sign != 0 else self._last_command_sign
            # Reset accumulated correction after a successful send
            self._pending_delta = 0.0
            return MoveCommand(delta_deg=p, sent=True, reason="sent")
        return MoveCommand(delta_deg=p, sent=False, reason="controller_rejected")