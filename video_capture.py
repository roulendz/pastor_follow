from typing import Optional, Tuple, Dict, Any
import cv2
import numpy as np
import queue
import threading
import time
import os


class VideoCapture:
    """High-FPS OpenCV capture with backend/fourcc tuning, watchdog, and dropout smoothing.

    Expects cfg keys: capture_index, width, height, fps, backend, fourcc, buffersize.
    Backward-compatible public API: get_frame(), get_status(), release().
    """

    def __init__(self, cfg: Dict[str, Any]):
        index = int(cfg.get('capture_index', 0))
        self.width = int(cfg.get('width', 1280))
        self.height = int(cfg.get('height', 720))
        self.target_fps = float(cfg.get('fps', 30))
        self.backend = (cfg.get('backend') or 'DSHOW').upper()
        self.fourcc = (cfg.get('fourcc') or 'MJPG').upper()
        # Queue buffersize controls frame drop behavior; use small size to reduce latency
        # Support both 'buffersize' and legacy 'buffer_size'
        self.buffersize = int(cfg.get('buffersize', cfg.get('buffer_size', 2)))
        # Smooth short dropouts by holding last frame for a brief time
        self.dropout_hold_ms = int(cfg.get('dropout_hold_ms', 200))
        # Watchdog thresholds
        self.reinit_fail_threshold = int(cfg.get('reinit_fail_threshold', 30))  # consecutive read failures
        self.reinit_low_fps_threshold = float(cfg.get('reinit_low_fps_threshold', 12.0))
        self.reinit_low_fps_window_s = float(cfg.get('reinit_low_fps_window_s', 3.0))

        # Set OpenCV optimizations
        try:
            cv2.setUseOptimized(True)
            cv2.setNumThreads(max(2, os.cpu_count() or 2))
        except Exception:
            pass

        # Runtime state
        self.cap: Optional[cv2.VideoCapture] = None
        self.synthetic = False
        self._queue: Optional[queue.Queue] = None
        self._running: bool = False
        self._capture_thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None
        self._last_frame: Optional[np.ndarray] = None
        self._last_frame_ts: float = 0.0
        self._fps_count: int = 0
        self._fps_start: float = time.time()
        self._fail_count: int = 0
        self._reinit_attempts: int = 0
        self._backend_active: str = self.backend
        self._drops_1s: int = 0
        self._drops_window_start: float = time.time()

        # Initialize resources
        self._queue = queue.Queue(maxsize=max(1, self.buffersize))
        self._running = True
        if not self._open_capture(index, self.backend):
            # Fallback to MSMF
            self._open_capture(index, 'MSMF')

        # Start threads
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def _open_capture(self, index: int, backend: str) -> bool:
        """Open camera with backend and apply tuned properties; return success."""
        try:
            backend_flag = cv2.CAP_DSHOW if backend.upper() == 'DSHOW' else (
                cv2.CAP_MSMF if backend.upper() == 'MSMF' else 0)
            cap = cv2.VideoCapture(index, backend_flag) if backend_flag else cv2.VideoCapture(index)
            if not cap or not cap.isOpened():
                return False
            # Properties: cast to ints where required
            try:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.width))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.height))
                cap.set(cv2.CAP_PROP_FPS, float(self.target_fps))
                fourcc_val = cv2.VideoWriter_fourcc(*self.fourcc)
                cap.set(cv2.CAP_PROP_FOURCC, fourcc_val)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, int(self.buffersize))
            except Exception:
                # Some backends ignore certain properties; proceed anyway
                pass
            # Swap in new capture
            self._close_capture()
            self.cap = cap
            self.synthetic = False
            self._backend_active = backend.upper()
            self._fail_count = 0
            return True
        except Exception:
            return False

    def _close_capture(self) -> None:
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass
        finally:
            self.cap = None

    def _capture_loop(self):
        while self._running:
            ok = False
            frame = None
            try:
                if self.cap:
                    ok, frame = self.cap.read()
                else:
                    ok, frame = False, None
            except Exception:
                ok, frame = False, None

            now = time.time()
            if ok and frame is not None:
                self._fps_count += 1
                self._fail_count = 0
                self._last_frame = frame
                self._last_frame_ts = now
                try:
                    if self._queue and self._queue.full():
                        _ = self._queue.get_nowait()
                    if self._queue:
                        self._queue.put_nowait(frame)
                except Exception:
                    pass
            else:
                # Count drop and optionally hold last frame briefly to reduce blinking
                self._fail_count += 1
                self._drops_1s += 1
                hold_ok = (self._last_frame is not None) and ((now - self._last_frame_ts) * 1000.0 < self.dropout_hold_ms)
                try:
                    if hold_ok and self._queue:
                        if self._queue.full():
                            _ = self._queue.get_nowait()
                        self._queue.put_nowait(self._last_frame)
                    elif self._queue:
                        if self._queue.full():
                            _ = self._queue.get_nowait()
                        self._queue.put_nowait(self._synthetic_frame())
                except Exception:
                    pass

            # Reset drops window every ~1s
            if (now - self._drops_window_start) >= 1.0:
                self._drops_window_start = now
                self._drops_1s = 0

    def _watchdog_loop(self):
        """Reinitialize capture when persistent failures or very low FPS are detected."""
        low_fps_start = 0.0
        while self._running:
            time.sleep(0.5)
            # Check low FPS over window
            fps_meas = self._current_fps()
            if fps_meas < self.reinit_low_fps_threshold:
                if low_fps_start == 0.0:
                    low_fps_start = time.time()
            else:
                low_fps_start = 0.0

            need_reinit = False
            if self._fail_count >= self.reinit_fail_threshold:
                need_reinit = True
            elif low_fps_start > 0.0 and (time.time() - low_fps_start) >= self.reinit_low_fps_window_s:
                need_reinit = True

            if need_reinit:
                self._reinit_attempts += 1
                # Try preferred backend first, then fallback
                idx = 0
                backends = [self.backend, ('MSMF' if self.backend != 'MSMF' else 'DSHOW')]
                ok = False
                for b in backends:
                    if self._open_capture(int(self.cap.get(cv2.CAP_PROP_DEVICE)) if self.cap else 0, b):
                        ok = True
                        break
                if not ok:
                    # Switch to synthetic frames until device recovers
                    self.synthetic = True
                # Reset counters after reinit attempt
                self._fail_count = 0
                low_fps_start = 0.0

    def _current_fps(self) -> float:
        now = time.time()
        elapsed = max(1e-3, now - self._fps_start)
        fps = float(self._fps_count) / elapsed
        if elapsed >= 1.0:
            self._fps_start = now
            self._fps_count = 0
        return fps

    def _synthetic_frame(self) -> np.ndarray:
        # Neutral grey canvas + overlay text
        frame = np.full((self.height, self.width, 3), 30, dtype=np.uint8)
        cv2.putText(frame, "Synthetic video: camera not available", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (240, 240, 240), 2, cv2.LINE_AA)
        return frame

    def get_frame(self, timeout: float = 0.05) -> Tuple[bool, Optional[object]]:
        if not self._running:
            return True, self._synthetic_frame()
        try:
            if self._queue:
                frame = self._queue.get(timeout=timeout)
            else:
                frame = None
            return True, (frame if frame is not None else self._synthetic_frame())
        except Exception:
            return True, self._synthetic_frame()

    def get_status(self) -> Dict[str, Any]:
        fps_measured = self._current_fps()
        return {
            'connected': bool(self.cap and self.cap.isOpened() and not self.synthetic),
            'resolution': (self.width, self.height),
            'fps_target': self.target_fps,
            'fps_measured': fps_measured,
            'backend': self._backend_active,
            'fourcc': self.fourcc,
            'fail_count': self._fail_count,
            'reinit_attempts': self._reinit_attempts,
            'drops_1s': self._drops_1s,
        }

    def release(self) -> None:
        # Gracefully stop threads and release resources
        try:
            self._running = False
            # Join threads
            try:
                if self._capture_thread:
                    self._capture_thread.join(timeout=0.5)
            except Exception:
                pass
            try:
                if self._watchdog_thread:
                    self._watchdog_thread.join(timeout=0.5)
            except Exception:
                pass
            # Release camera
            self._close_capture()
            # Clear queue
            try:
                if self._queue:
                    while not self._queue.empty():
                        _ = self._queue.get_nowait()
            except Exception:
                pass
        except Exception:
            pass