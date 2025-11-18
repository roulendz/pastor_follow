from typing import Optional, Tuple, Dict, Any
import cv2
import numpy as np
import queue
import threading
import time
import os


class VideoCapture:
    """High-FPS OpenCV capture with backend/fourcc tuning and threaded buffer.

    Expects cfg keys: capture_index, width, height, fps, backend, fourcc, buffersize.
    """

    def __init__(self, cfg: Dict[str, Any]):
        index = int(cfg.get('capture_index', 0))
        self.width = int(cfg.get('width', 1280))
        self.height = int(cfg.get('height', 720))
        self.target_fps = float(cfg.get('fps', 30))
        self.backend = (cfg.get('backend') or 'DSHOW').upper()
        self.fourcc = (cfg.get('fourcc') or 'MJPG').upper()
        self.buffersize = int(cfg.get('buffersize', 1))

        # Set OpenCV optimizations
        try:
            cv2.setUseOptimized(True)
            cv2.setNumThreads(max(2, os.cpu_count() or 2))
        except Exception:
            pass

        # Select backend
        backend_flag = 0
        if self.backend == 'DSHOW':
            backend_flag = cv2.CAP_DSHOW
        elif self.backend == 'MSMF':
            backend_flag = cv2.CAP_MSMF
        else:
            backend_flag = 0

        self.cap = cv2.VideoCapture(index, backend_flag) if backend_flag else cv2.VideoCapture(index)
        self.synthetic = False
        try:
            opened = bool(self.cap.isOpened())
        except Exception:
            opened = False

        if opened:
            # Try tuned properties
            try:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
                self.cap.set(cv2.CAP_PROP_FPS, float(self.target_fps))
                # Fourcc tuning for 60fps webcams
                fourcc_val = cv2.VideoWriter_fourcc(*self.fourcc)
                self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_val)
                # Buffer size (not supported by all backends)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, float(self.buffersize))
            except Exception:
                pass

            # Threaded latest-frame buffer
            self._queue = queue.Queue(maxsize=1)
            self._running = True
            self._fps_count = 0
            self._fps_start = time.time()
            threading.Thread(target=self._capture_loop, daemon=True).start()
        else:
            self.synthetic = True
            self._running = False
            self._queue = None
            self._fps_count = 0
            self._fps_start = time.time()

    def _capture_loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                # If camera fails, switch to synthetic
                self.synthetic = True
                break
            self._fps_count += 1
            # keep latest frame only
            try:
                if self._queue.full():
                    _ = self._queue.get_nowait()
                self._queue.put_nowait(frame)
            except Exception:
                pass

    def _synthetic_frame(self) -> np.ndarray:
        # Neutral grey canvas + overlay text
        frame = np.full((self.height, self.width, 3), 30, dtype=np.uint8)
        cv2.putText(frame, "Synthetic video: camera not available", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (240, 240, 240), 2, cv2.LINE_AA)
        return frame

    def get_frame(self, timeout: float = 0.05) -> Tuple[bool, Optional[object]]:
        if self.synthetic or not self._running:
            return True, self._synthetic_frame()
        try:
            frame = self._queue.get(timeout=timeout)
            return True, frame
        except Exception:
            # If no frame available, generate synthetic to keep UI responsive
            return True, self._synthetic_frame()

    def get_status(self) -> Dict[str, Any]:
        now = time.time()
        elapsed = max(1e-3, now - self._fps_start)
        measured_fps = float(self._fps_count) / elapsed
        # reset window every ~1s for stability
        if elapsed >= 1.0:
            self._fps_start = now
            self._fps_count = 0
        return {
            'connected': not self.synthetic,
            'resolution': (self.width, self.height),
            'fps_target': self.target_fps,
            'fps_measured': measured_fps,
            'backend': self.backend,
            'fourcc': self.fourcc,
        }

    def release(self) -> None:
        try:
            self._running = False
            if not self.synthetic:
                self.cap.release()
        except Exception:
            pass