from __future__ import annotations
from typing import Tuple

from .errors import CoordinateError


class CoordinateMapper:
    """Maps raw normalized coordinates into calibrated space and computes error.

    Responsibilities:
    - Apply horizontal inversion per calibration
    - Apply center offset
    - Clamp to configured bounds
    - Compute error relative to PID setpoint
    - Evaluate center deadband (normalized)
    """

    def __init__(self, config):
        self.config = config

    def map_x(self, x: float) -> Tuple[float, bool]:
        try:
            invert = bool(self.config.get('calibration', 'invert_horizontal'))
            offset = float(self.config.get('calibration', 'center_offset_x') or 0.0)
            clamp_min = float(self.config.get('calibration', 'x_clamp_min') or 0.0)
            clamp_max = float(self.config.get('calibration', 'x_clamp_max') or 1.0)
        except Exception as e:
            raise CoordinateError(f"invalid calibration config: {e}")

        if not (0.0 <= x <= 1.0):
            raise CoordinateError(f"x out of bounds: {x}")

        x_cal = 1.0 - x if invert else x
        x_cal += offset
        # Clamp
        if clamp_min > clamp_max:
            raise CoordinateError("clamp_min > clamp_max")
        x_cal = max(clamp_min, min(clamp_max, x_cal))

        # Deadband eval needs setpoint and normalized width value
        try:
            center_db_norm = float(self.config.get('tracking', 'center_deadband_norm') or 0.0)
            setpoint = float(self.config.get('pid', 'setpoint') or 0.5)
        except Exception as e:
            raise CoordinateError(f"invalid tracking/pid config: {e}")

        err = setpoint - x_cal
        deadband_active = abs(err) < center_db_norm
        return x_cal, deadband_active