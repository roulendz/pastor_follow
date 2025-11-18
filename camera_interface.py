from typing import Tuple, Any
import time

from video_capture import VideoCapture


class CameraInterface:
    def __init__(self, ob_video_capture: VideoCapture):
        self.ob_video_capture = ob_video_capture

    def grab_frame(self) -> Tuple[Any, float]:
        b_ok, ob_frame = self.ob_video_capture.get_frame()
        f_timestamp_s = time.time()
        return ob_frame, f_timestamp_s