import math
from dataclasses import dataclass
from typing import Optional

from sample import Sample

@dataclass
class FovEstimatorConfig:
    float_initial_angle_per_pixel: float = 0.05
    float_min_angle_degrees: float = 1.0
    float_min_pixels: float = 10.0
    float_alpha: float = 0.05

class FovEstimator:
    """
    Estimates anglePerPixel and field of view based on motor and person movement.
    """

    def __init__(self, config: FovEstimatorConfig = FovEstimatorConfig()):
        self.config = config
        self.reset(self.config.float_initial_angle_per_pixel)

    def reset(self, float_initial_angle_per_pixel: float) -> None:
        """
        Resets the estimator with an initial angle per pixel value.
        """
        self.float_angle_per_pixel_estimated = float_initial_angle_per_pixel
        self.boolean_has_previous_sample = False
        self.previous_sample: Optional[Sample] = None

    def update_with_sample(self, sample: Sample) -> None:
        """
        Updates the FOV estimate with a new sample.
        """
        if not sample.boolean_person_valid or sample.float_x_person_pixel is None:
            self.boolean_has_previous_sample = False
            return

        if not self.boolean_has_previous_sample:
            self.previous_sample = sample
            self.boolean_has_previous_sample = True
            return

        float_delta_angle = sample.float_motor_angle_degrees - self.previous_sample.float_motor_angle_degrees
        float_delta_x = sample.float_x_person_pixel - self.previous_sample.float_x_person_pixel

        # Only learn when there is significant motion
        if math.fabs(float_delta_angle) > self.config.float_min_angle_degrees and \
           math.fabs(float_delta_x) > self.config.float_min_pixels:
            float_sample_angle_per_pixel = float_delta_angle / float_delta_x

            # Exponential moving average (smooth+stable)
            self.float_angle_per_pixel_estimated = (
                (1.0 - self.config.float_alpha) * self.float_angle_per_pixel_estimated +
                self.config.float_alpha * float_sample_angle_per_pixel
            )

            self.previous_sample = sample

    def get_angle_per_pixel(self) -> float:
        """
        Returns the estimated angle per pixel.
        """
        return self.float_angle_per_pixel_estimated

    def get_fov_degrees(self, integer_image_width: int) -> float:
        """
        Returns the estimated field of view in degrees.
        """
        return self.float_angle_per_pixel_estimated * integer_image_width