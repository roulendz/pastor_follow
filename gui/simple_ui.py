from __future__ import annotations
import time
import threading
import queue
import logging
from typing import Optional

import cv2
import numpy as np
from PIL import Image
import customtkinter as ctk
from customtkinter import CTkImage

from config import Config
from video_capture import VideoCapture
from arduino_controller import ArduinoController
from raw_tracking import RawMovementController, RawSessionLogger
from raw_tracking.slider_logger import SliderCSVLogger
from raw_tracking.slider_motion import SliderMotionController, MotionConfig, PIDConfig, map_deg_to_norm, plan_center_animation_steps


class SimpleTrackingApp(ctk.CTk):
    """Minimal UI: one slider + live view + offset label.

    Purpose-built for raw tracking without calibration transforms.
    """

    def __init__(self):
        super().__init__()
        self.title("Human Tracking System - Raw Control")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Configure logging
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', filename='f:\\Documents\\Arduino\\pastor_follow_v2\\last_session.log', filemode='a')
        self.app_logger = logging.getLogger(__name__)
        self.app_logger.info("Application started.")

        # Config and hardware
        self.cfg = Config()
        # Make host-side rate-limiting permissive for immediacy
        self.cfg.set('arduino', 'min_command_interval_ms', 0.0)
        self.cfg.set('arduino', 'command_deadband_deg', 0.0)

        # Loggers: human-readable (failures) and CSV telemetry
        # Write human-readable logs to a separate file to keep CSV clean
        self.logger = RawSessionLogger(sLogPath="f:\\Documents\\Arduino\\pastor_follow_v2\\session_info.log")
        self.csv_logger = SliderCSVLogger(
            log_path="f:\\Documents\\Arduino\\pastor_follow_v2\\last_session.log",
            slow_threshold_deg_s=float(self.cfg.get('tracking', 'slow_threshold_deg_s') or 1.0),
            engage_threshold_norm=float(self.cfg.get('tracking', 'engage_threshold_norm') or 0.2),
            unidir_hold_ms=int(self.cfg.get('tracking', 'unidir_hold_ms') or 250),
            write_header=True,
        )
        # VideoCapture expects a config dict
        self.video_capture = VideoCapture({
            'capture_index': self.cfg.get('video', 'capture_index'),
            'width': self.cfg.get('video', 'width'),
            'height': self.cfg.get('video', 'height'),
            'fps': self.cfg.get('video', 'fps'),
            'backend': self.cfg.get('video', 'backend'),
            'fourcc': self.cfg.get('video', 'fourcc'),
            'buffersize': self.cfg.get('video', 'buffersize'),
        })
        self.arduino_controller = ArduinoController(self.cfg.get('arduino'))
        # Establish connection so immediate OutDelta commands move the motor
        try:
            self.arduino_controller.connect()
        except Exception as e:
            self.app_logger.error(f"Failed to connect to Arduino: {e}")
            pass

        # Initialize new tracking components
        self.motor_interface = MotorInterface(self.arduino_controller)
        self.camera_interface = CameraInterface(self.video_capture)
        self.pose_tracker = PoseTracker()
        self.fov_estimator = FOVEstimator(
            initial_fov_deg=float(self.cfg.get('tracking', 'initial_fov_deg') or 60.0),
            alpha=float(self.cfg.get('tracking', 'fov_alpha') or 0.1)
        )
        self.tracker_controller = TrackerController(
            camera_interface=self.camera_interface,
            motor_interface=self.motor_interface,
            pose_tracker=self.pose_tracker,
            fov_estimator=self.fov_estimator,
            logger=self.app_logger,
            pid_kp=float(self.cfg.get('pid', 'kp') or 1.0),
            pid_ki=float(self.cfg.get('pid', 'ki') or 0.0),
            pid_kd=float(self.cfg.get('pid', 'kd') or 0.0),
            pid_i_limit=float(self.cfg.get('pid', 'i_limit') or 180.0),
            motor_speed_limit_deg_s=float(self.cfg.get('motion', 'max_speed_deg_s') or 60.0)
        )

        # Raw mover (kept for future head-tracking), but slider will control motion now
        self.mover = RawMovementController(
            obArduino=self.arduino_controller,
            fScaleDegPerUnit=float(self.cfg.get('pid', 'output_scale_deg') or 60.0),
            fDeadbandDeg=float(self.cfg.get('arduino', 'command_deadband_deg') or 0.0),
            logger=self.logger,
            fnOnSend=self._on_send,
        )
        # Motion controller for slider-based control
        self.motion = SliderMotionController(
            cfg=MotionConfig(
                max_speed_deg_s=float(self.cfg.get('motion', 'max_speed_deg_s') or 60.0),
                curve_exponent=float(self.cfg.get('motion', 'curve_exponent') or 1.6),
                deadzone_norm=float(self.cfg.get('motion', 'deadzone_norm') or 0.05),
                ease_tau_s=float(self.cfg.get('motion', 'ease_tau_s') or 0.2),
                max_accel_deg_s2=float(self.cfg.get('motion', 'max_accel_deg_s2') or 240.0),
                auto_return_enabled=bool(self.cfg.get('motion', 'auto_return_enabled') if self.cfg.get('motion', 'auto_return_enabled') is not None else True),
                return_idle_ms=int(self.cfg.get('motion', 'return_idle_ms') or 120),
                return_speed_deg_s=float(self.cfg.get('motion', 'return_speed_deg_s') or 40.0),
                min_command_deg=float(self.cfg.get('motion', 'min_command_deg') or 0.02),
            ),
            pid_cfg=PIDConfig(
                kp=float(self.cfg.get('pid', 'kp') or 1.0),
                ki=float(self.cfg.get('pid', 'ki') or 0.0),
                kd=float(self.cfg.get('pid', 'kd') or 0.0),
                i_limit=float(self.cfg.get('pid', 'i_limit') or 180.0),
            ),
        )

        # UI elements
        # Control range and center
        self.range_deg = int(self.cfg.get('control', 'range_deg') or 90)
        self.center_deg = int(self.cfg.get('control', 'center_deg') or 45)
        # Slider mode: 'position' for absolute angle, 'velocity' for speed-based control
        self.slider_mode = str(self.cfg.get('control', 'slider_mode') or 'velocity')

        # Layout stretch: make columns expand so slider can be full width
        try:
            self.grid_columnconfigure(0, weight=1)
            self.grid_columnconfigure(1, weight=1)
            self.grid_columnconfigure(2, weight=1)
        except Exception:
            pass

        # Slider shows offset degrees relative to center: [-range/2, +range/2]
        self.slider = ctk.CTkSlider(self, from_=-float(self.range_deg)/2.0, to=float(self.range_deg)/2.0, command=self._on_slider)
        # Place slider across the full window width
        self.slider.grid(row=0, column=0, columnspan=3, padx=10, pady=8, sticky="ew")
        # Center is 0 offset
        self.slider.set(0.0)

        # Offset label and HOME button on the next row
        self.lbl_offset = ctk.CTkLabel(self, text=f"Offset: 0.00°", font=("Arial", 16))
        self.lbl_offset.grid(row=1, column=0, sticky="w", padx=10, pady=6)

        self.btn_home = ctk.CTkButton(self, text="HOME", command=self._on_home)
        self.btn_home.grid(row=1, column=2, padx=10, pady=6, sticky="e")

        self.lbl_canvas = ctk.CTkLabel(self, text="")
        self.lbl_canvas.grid(row=2, column=0, columnspan=3, padx=10, pady=10)

        # Video status indicators
        self.lbl_video = ctk.CTkLabel(self, text="Video: init", font=("Arial", 12))
        self.lbl_video.grid(row=3, column=0, columnspan=3, sticky="w", padx=10, pady=6)

        # Motor status indicator
        self.lbl_motor = ctk.CTkLabel(self, text="Motor: init", font=("Arial", 12))
        self.lbl_motor.grid(row=4, column=0, columnspan=3, sticky="w", padx=10, pady=6)

        # Keep slider width in sync with window size for granularity
        try:
            self.bind("<Configure>", self._on_resize)
        except Exception:
            pass

        # Track both offset and absolute degrees
        self._last_slider_offset = 0.0
        self._last_slider_abs = float(self.center_deg)
        self._running = True
        self._last_loop_ts = time.perf_counter()
        self._fps_ts = time.perf_counter()
        self._fps_frames = 0
        # UI thread-safety helpers
        self._ui_queue = queue.Queue(maxsize=2)
        self._home_anim_running = False
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()
        # Pump queued UI updates on the main thread
        self.after(16, self._ui_pump)

    def _on_send(self, fDeltaDeg: float):
        # Hook to observe sends; no-op for UI
        pass

    def _on_slider(self, fVal: float):
        # Slider now outputs offset degrees relative to center: [-range/2, +range/2]
        offset = float(fVal)
        self._last_slider_offset = offset
        deg_abs = float(self.center_deg) + offset
        self._last_slider_abs = deg_abs
        # Map absolute degree to normalized [-1,1] around center
        norm = map_deg_to_norm(deg_abs, self.center_deg, self.range_deg)
        try:
            if self.slider_mode == 'position':
                # Send absolute stage angle to Arduino and neutralize motion controller
                ok = bool(self.arduino.move_to_abs(deg_abs))
                try:
                    self.lbl_motor.configure(text=f"Motor: {'connected' if self.arduino._ser else 'stub'} | cmd={'OK' if ok else 'FAIL'} | tgt={deg_abs:.2f}°")
                except Exception:
                    pass
                self.motion.set_user_input(0.0)
            else:
                # Velocity mode: drive motion controller from normalized input
                self.motion.set_user_input(norm)
            self.lbl_offset.configure(text=f"Offset: {offset:.2f}°")
        except Exception as e:
            self.logger.log_failure(f"slider_input_error:{e}")

    def _on_home(self):
        if self._home_anim_running:
            return
        self._home_anim_running = True
        # Visual feedback
        try:
            self.btn_home.configure(state="disabled", text="HOMING…")
        except Exception:
            pass
        # In position mode, issue absolute move to center immediately
        if self.slider_mode == 'position':
            try:
                self.arduino.move_to_abs(self.center_deg)
            except Exception as e:
                self.logger.log_failure(f"home_move_error:{e}")
        # Animate slider back to center
        duration = int(self.cfg.get('control', 'home_anim_ms') or 250)
        steps = plan_center_animation_steps(self._last_slider_abs, self.center_deg, duration_ms=duration)
        def _step(i=0):
            if i >= len(steps):
                # Reset last slider tracking to center
                self._last_slider_offset = 0.0
                self._last_slider_abs = float(self.center_deg)
                try:
                    self.btn_home.configure(state="normal", text="HOME")
                except Exception:
                    pass
                self._home_anim_running = False
                return
            deg = steps[i]
            try:
                # Convert absolute step to offset for the UI slider
                offset = float(deg) - float(self.center_deg)
                self.slider.set(offset)
                # Update controller input live during animation
                norm = map_deg_to_norm(deg, self.center_deg, self.range_deg)
                if self.slider_mode == 'position':
                    # In position mode, motion controller is neutralized; Arduino already moving
                    pass
                else:
                    self.motion.set_user_input(norm)
                self.lbl_offset.configure(text=f"Offset: {offset:.2f}°")
            except Exception as e:
                self.logger.log_failure(f"home_anim_error:{e}")
            self.after(max(1, int(duration / max(1, len(steps)))), lambda: _step(i + 1))

        _step(0)

    def _loop(self):
        while self._running:
            # Compute dt for motion controller
            now = time.perf_counter()
            dt = max(0.0, now - self._last_loop_ts)
            self._last_loop_ts = now

            # Use TrackerController to perform one tick of tracking and control
            try:
                self.tracker_controller.tick()
                current_sample = self.tracker_controller.get_latest_sample()
                if current_sample and current_sample.person_valid and current_sample.x_person_pixel_position is not None:
                    # For UI, we need the frame and potentially annotated frame
                    # The TrackerController's tick method should handle frame grabbing and pose estimation
                    # We need to get the frame from the camera_interface after the tick
                    _, frame = self.camera_interface.grab_frame()
                    if frame is not None:
                        # Draw a circle at the detected person's x-position for visual feedback
                        frame_height, frame_width, _ = frame.shape
                        x_pixel = int(current_sample.x_person_pixel_position)
                        cv2.circle(frame, (x_pixel, frame_height // 2), 20, (0, 255, 0), 2) # Green circle

                        if self._ui_queue.full():
                            _ = self._ui_queue.get_nowait()
                        self._ui_queue.put_nowait({'frame': frame, 'w': frame_width, 'h': frame_height, 'fps': 0.0, 'connected': True, 'backend': 'OpenCV', 'drops': 0, 'fails': 0})
                else:
                    # If no person detected or frame not available, still try to get a frame for display
                    _, frame = self.camera_interface.grab_frame()
                    if frame is not None:
                        frame_height, frame_width, _ = frame.shape
                        if self._ui_queue.full():
                            _ = self._ui_queue.get_nowait()
                        self._ui_queue.put_nowait({'frame': frame, 'w': frame_width, 'h': frame_height, 'fps': 0.0, 'connected': True, 'backend': 'OpenCV', 'drops': 0, 'fails': 0})
                    else:
                        if self._ui_queue.full():
                            _ = self._ui_queue.get_nowait()
                        self._ui_queue.put_nowait({'frame': None, 'w': 0, 'h': 0, 'fps': 0.0})

                # Update motor status label based on motor_interface feedback
                motor_state = self.motor_interface.get_latest_state()
                if motor_state:
                    self.lbl_motor.configure(text=f"Motor: connected | angle={motor_state.motor_angle_degrees:.2f}°")
                else:
                    self.lbl_motor.configure(text="Motor: disconnected")

            except Exception as e:
                self.app_logger.error(f"Error in main loop: {e}")

            time.sleep(0.01)

    def _ui_pump(self):
        # Consume queued video updates and apply them on the UI thread
        try:
            item = self._ui_queue.get_nowait()
        except Exception:
            item = None

        if item is not None:
            frame = item.get('frame')
            if frame is not None:
                try:
                    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    im = Image.fromarray(img_rgb)
                    im_ctk = CTkImage(light_image=im, dark_image=im, size=(img_rgb.shape[1]//2, img_rgb.shape[0]//2))
                    self.lbl_canvas.configure(image=im_ctk)
                    self.lbl_canvas.image = im_ctk
                    w = int(item.get('w', img_rgb.shape[1]))
                    h = int(item.get('h', img_rgb.shape[0]))
                    fps_meas = float(item.get('fps', 0.0))
                    connected = bool(item.get('connected', False))
                    backend = str(item.get('backend', ''))
                    drops = int(item.get('drops', 0))
                    fails = int(item.get('fails', 0))
                    if connected:
                        self.lbl_video.configure(text=f"Video: {backend} {w}x{h} @ {fps_meas:.1f}fps | drops:{drops} | fails:{fails}")
                    else:
                        self.lbl_video.configure(text=f"Video: disconnected (showing synthetic)")
                except Exception as e:
                    self.lbl_video.configure(text=f"Video error: {e}")
            else:
                self.lbl_video.configure(text="Video: disconnected")

        # Schedule next pump
        self.after(16, self._ui_pump)

    def _telemetry_loop(self):
        # Dedicated telemetry logging at ~50Hz
        while self._running:
            fb = self.arduino.get_feedback(timeout=0.0)
            angle = float(fb.current_angle) if fb else float(self.arduino.current_position or 0.0)
            speed = float(fb.current_speed) if fb else 0.0
            try:
                self.csv_logger.log_sample(
                    slider_norm=float(self.motion.user_norm),
                    motor_angle_deg=float(angle),
                    motor_speed_deg_s=float(speed),
                    slider_offset_deg=float(self._last_slider_offset),
                    target_angle_deg=float(self._last_slider_abs),
                    slider_mode=str(self.slider_mode),
                )
            except Exception as e:
                self.logger.log_failure(f"csv_log_error:{e}")
            time.sleep(0.02)

    def _on_resize(self, event):
        try:
            # Estimate interior width minus padding for a responsive slider
            width = max(100, self.winfo_width() - 24)
            self.slider.configure(width=width)
        except Exception:
            pass

    def on_close(self):
        self._running = False
        # Give threads a moment to exit cleanly
        time.sleep(0.1)
        self.logger.close()
        try:
            self.csv_logger.close()
        except Exception:
            pass
        try:
            self.video_capture.release()
        except Exception:
            pass
        try:
            self.arduino_controller.disconnect()
        except Exception:
            pass
        self.app_logger.info("Application closed.")
        self.destroy()


def run_app():
    app = SimpleTrackingApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()