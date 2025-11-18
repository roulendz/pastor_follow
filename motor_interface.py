from dataclasses import dataclass
from typing import Any, Dict
import time

from arduino_controller import ArduinoController, Feedback


@dataclass
class MotorState:
    f_angle_deg: float
    f_timestamp_s: float


class MotorInterface:
    def __init__(self, ob_arduino_controller: ArduinoController):
        self.ob_arduino_controller = ob_arduino_controller

    def get_latest_state(self) -> MotorState:
        fb_feedback = self.ob_arduino_controller.get_feedback()
        if fb_feedback:
            return MotorState(
                f_angle_deg=fb_feedback.current_angle,
                f_timestamp_s=fb_feedback.timestamp_ms / 1000.0
            )
        else:
            # Fallback if no feedback is available, use current position and current time
            return MotorState(
                f_angle_deg=self.ob_arduino_controller.current_position,
                f_timestamp_s=time.time()
            )