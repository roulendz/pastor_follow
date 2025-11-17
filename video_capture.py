"""
Video Capture Module
Handles video input from capture cards with threading for non-blocking operation
"""

import cv2
import threading
import queue
import time
import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass
from enum import Enum

class CaptureState(Enum):
    STOPPED = 0
    STARTING = 1
    RUNNING = 2
    STOPPING = 3
    ERROR = 4

@dataclass
class VideoDevice:
    index: int
    name: str
    backend: str
    available: bool

class VideoCapture:
    """Thread-safe video capture with buffering"""
    
    def __init__(self, config: dict):
        self.config = config
        self.capture = None
        self.thread = None
        self.state = CaptureState.STOPPED
        self.frame_queue = queue.Queue(maxsize=config.get('buffer_size', 1))
        self.stats_queue = queue.Queue(maxsize=10)
        self.lock = threading.Lock()
        
        # Performance tracking
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.dropped_frames = 0
        
        # Current device info
        self.current_device = None
        self.device_list = []
        
    @staticmethod
    def list_devices() -> List[VideoDevice]:
        """List all available video capture devices"""
        devices = []
        
        # Windows-specific backends to try
        backends = [
            (cv2.CAP_DSHOW, "DirectShow"),
            (cv2.CAP_MSMF, "Media Foundation"),
            (cv2.CAP_ANY, "Any")
        ]
        
        # Test up to 10 devices
        for i in range(10):
            for backend_id, backend_name in backends:
                cap = cv2.VideoCapture(i, backend_id)
                if cap.isOpened():
                    # Try to get device name (Windows-specific)
                    ret, _ = cap.read()
                    if ret:
                        devices.append(VideoDevice(
                            index=i,
                            name=f"Device {i} ({backend_name})",
                            backend=backend_name,
                            available=True
                        ))
                        cap.release()
                        break
                cap.release()
        
        return devices
    
    def start(self, device_index: int = None, backend: int = cv2.CAP_DSHOW) -> bool:
        """Start video capture from specified device"""
        if self.state != CaptureState.STOPPED:
            return False
        
        self.state = CaptureState.STARTING
        device_index = device_index or self.config['capture_index']
        
        try:
            # Open capture device
            self.capture = cv2.VideoCapture(device_index, backend)
            
            if not self.capture.isOpened():
                self.state = CaptureState.ERROR
                return False
            
            # Set capture properties
            self._configure_capture()
            
            # Clear queues
            self._clear_queues()
            
            # Start capture thread
            self.thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.thread.start()
            
            # Wait for first frame
            timeout = 5  # seconds
            start_time = time.time()
            while self.frame_queue.empty() and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.frame_queue.empty():
                self.stop()
                self.state = CaptureState.ERROR
                return False
            
            self.state = CaptureState.RUNNING
            self.current_device = device_index
            return True
            
        except Exception as e:
            print(f"Error starting capture: {e}")
            self.state = CaptureState.ERROR
            return False
    
    def stop(self):
        """Stop video capture"""
        if self.state == CaptureState.STOPPED:
            return
        
        self.state = CaptureState.STOPPING
        
        # Wait for thread to finish
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        
        # Release capture
        if self.capture:
            self.capture.release()
            self.capture = None
        
        self._clear_queues()
        self.state = CaptureState.STOPPED
        self.current_device = None
    
    def get_frame(self, timeout: float = 0.1) -> Tuple[bool, Optional[np.ndarray]]:
        """Get the latest frame from the capture buffer"""
        if self.state != CaptureState.RUNNING:
            return False, None
        
        try:
            frame = self.frame_queue.get(timeout=timeout)
            return True, frame
        except queue.Empty:
            return False, None
    
    def get_stats(self) -> dict:
        """Get capture statistics"""
        stats = {
            'fps': self.fps,
            'frame_count': self.frame_count,
            'dropped_frames': self.dropped_frames,
            'state': self.state.name,
            'queue_size': self.frame_queue.qsize()
        }
        
        # Get device-specific stats if available
        if self.capture and self.capture.isOpened():
            stats['actual_fps'] = self.capture.get(cv2.CAP_PROP_FPS)
            stats['width'] = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            stats['height'] = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        return stats
    
    def set_resolution(self, width: int, height: int) -> bool:
        """Change capture resolution"""
        if not self.capture or not self.capture.isOpened():
            return False
        
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Verify the change
        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        return actual_width == width and actual_height == height
    
    def _configure_capture(self):
        """Configure capture device settings"""
        if not self.capture:
            return
        
        # Set resolution
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['width'])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['height'])
        
        # Set FPS
        self.capture.set(cv2.CAP_PROP_FPS, self.config['fps'])
        
        # Set buffer size (minimize latency)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # DirectShow specific settings for Windows
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    def _capture_loop(self):
        """Main capture loop running in separate thread"""
        while self.state in [CaptureState.STARTING, CaptureState.RUNNING]:
            try:
                ret, frame = self.capture.read()
                
                if not ret:
                    continue
                
                # Update FPS counter
                self._update_fps()
                
                # Add frame to queue (non-blocking)
                try:
                    # Remove old frame if queue is full
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                            self.dropped_frames += 1
                        except queue.Empty:
                            pass
                    
                    self.frame_queue.put_nowait(frame)
                except queue.Full:
                    self.dropped_frames += 1
                
            except Exception as e:
                print(f"Capture error: {e}")
                self.state = CaptureState.ERROR
                break
    
    def _update_fps(self):
        """Update FPS calculation"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def _clear_queues(self):
        """Clear all queues"""
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        
        while not self.stats_queue.empty():
            try:
                self.stats_queue.get_nowait()
            except queue.Empty:
                break


class AsyncDeviceScanner:
    """Asynchronously scan for video devices without blocking"""
    
    def __init__(self, callback=None):
        self.callback = callback
        self.scanning = False
        self.devices = []
        
    def scan_async(self):
        """Start asynchronous device scan"""
        if self.scanning:
            return
        
        self.scanning = True
        thread = threading.Thread(target=self._scan_worker, daemon=True)
        thread.start()
    
    def _scan_worker(self):
        """Worker thread for device scanning"""
        try:
            self.devices = VideoCapture.list_devices()
            if self.callback:
                self.callback(self.devices)
        finally:
            self.scanning = False
