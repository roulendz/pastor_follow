import math
import time
from typing import Optional

import cv2

from camera_interface import CameraInterface
from motor_interface import MotorInterface
from pose_tracker import PoseTracker, PoseResult
from sample import Sample
from fov_estimator import FovEstimator, FovEstimatorConfig

class TrackerController:
    """
    High-level brain: builds Sample, updates FOV estimate, computes angle error, sends motor commands.
    Uses MotorInterface + CameraInterface + PoseTracker + FovEstimator.
    """

    def __init__(self,
                 camera_interface: CameraInterface,
                 motor_interface: MotorInterface,
                 pose_tracker: PoseTracker,
                 fov_estimator: FovEstimator,
                 image_width: int,
                 min_angle_degrees: float = 0.0,
                 max_angle_degrees: float = 180.0,
                 proportional_gain: float = 1.0,
                 deadband_degrees: float = 0.3):
        self.camera_interface = camera_interface
        self.motor_interface = motor_interface
        self.pose_tracker = pose_tracker
        self.fov_estimator = fov_estimator
        self.integer_image_width = image_width
        self.float_min_angle_degrees = min_angle_degrees
        self.float_max_angle_degrees = max_angle_degrees
        self.float_proportional_gain = proportional_gain
        self.float_deadband_degrees = deadband_degrees

    def tick(self) -> None:
        """
        Executes one step of the tracking and control loop.
        """
        # 1. Grab frame + timestamp
        numpy_frame, float_frame_timestamp = self.camera_interface.grab_frame()

        if numpy_frame is None:
            print("Warning: No frame received from camera.")
            return

        # 2. Get current motor angle (snapshot)
        motor_state = self.motor_interface.get_latest_state()

        # 3. Run pose estimation
        pose_result: PoseResult = self.pose_tracker.estimate_pose(numpy_frame)

        # 4. Build sample
        sample = Sample(
            float_timestamp=float_frame_timestamp,
            float_motor_angle_degrees=motor_state.float_angle_degrees,
            boolean_person_valid=pose_result.float_confidence > 0.5,
            float_x_person_pixel=pose_result.float_x_person_pixel
        )

        # 5. Update FOV estimator (learning loop)
        self.fov_estimator.update_with_sample(sample)

        # 6. Compute control action (centering loop)
        self._control_from_sample(sample)

    def _control_from_sample(self, sample: Sample) -> None:
        """
        Computes the control action based on the sample and sends motor commands.
        """
        if not sample.boolean_person_valid or sample.float_x_person_pixel is None:
            # No person detected, or invalid pixel data, so do nothing.
            return

        float_angle_per_pixel = self.fov_estimator.get_angle_per_pixel()
        float_center_x_pixel = self.integer_image_width * 0.5
        float_pixel_offset = sample.float_x_person_pixel - float_center_x_pixel
        float_angle_error_degrees = float_pixel_offset * float_angle_per_pixel

        # Apply deadband
        if math.fabs(float_angle_error_degrees) < self.float_deadband_degrees:
            float_angle_error_degrees = 0.0

        # P control on angle
        float_correction_degrees = self.float_proportional_gain * float_angle_error_degrees

        float_target_angle_degrees = sample.float_motor_angle_degrees + float_correction_degrees

        # Clamp target angle to valid range
        float_target_angle_degrees = max(self.float_min_angle_degrees, min(self.float_max_angle_degrees, float_target_angle_degrees))

        self.motor_interface.send_target_angle(float_target_angle_degrees)