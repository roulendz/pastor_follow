from dataclasses import dataclass
import cv2
import mediapipe as mp
from typing import Tuple, Optional

@dataclass
class PoseResult:
    float_x_person_pixel: Optional[float]
    float_confidence: float

class PoseTracker:
    """
    A class that wraps MediaPipe's pose detection to estimate the horizontal position of a person.
    """

    def __init__(self):
        self.pose_detector = mp.solutions.pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def estimate_pose(self, numpy_frame: cv2.Mat) -> PoseResult:
        """
        Estimates the horizontal position of a person in the given frame.

        Args:
            numpy_frame: The input frame as a NumPy array (OpenCV format).

        Returns:
            A PoseResult object containing the horizontal pixel position and confidence.
        """
        # Convert the BGR image to RGB.
        rgb_frame = cv2.cvtColor(numpy_frame, cv2.COLOR_BGR2RGB)
        
        # Process the image and find poses.
        results = self.pose_detector.process(rgb_frame)

        float_x_person_pixel = None
        float_confidence = 0.0

        if results.pose_landmarks:
            # Assuming we are interested in the nose landmark for horizontal position
            # The x-coordinate is normalized to [0.0, 1.0]
            nose_landmark = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.NOSE]
            
            # Convert normalized x to pixel coordinates
            image_width = numpy_frame.shape[1]
            float_x_person_pixel = nose_landmark.x * image_width
            
            # MediaPipe doesn't directly provide a confidence score per landmark,
            # but we can infer confidence from the presence of landmarks.
            # For simplicity, we'll set a high confidence if landmarks are detected.
            float_confidence = 0.9

        return PoseResult(float_x_person_pixel=float_x_person_pixel, float_confidence=float_confidence)