from dataclasses import dataclass
from typing import Optional

@dataclass
class Sample:
    float_timestamp: float
    float_motor_angle_degrees: float
    boolean_person_valid: bool
    float_x_person_pixel: Optional[float]