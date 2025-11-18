from __future__ import annotations
import time
import threading
import queue
from typing import Optional

import cv2
import numpy as np
from PIL import Image
import customtkinter as ctk
from customtkinter import CTkImage

from config import Config
from video_capture import VideoCapture
from pose_detection import PoseDetector, PositionSmoother
from arduino_controller import ArduinoController
from raw_tracking import RawMovementController, RawSessionLogger, RawHeadTrackingPipeline


class SimpleTrackingApp(ctk.CTk):
    """Minimal UI: one slider + live view + offset label.

    Purpose-built for raw tracking without calibration transforms.
    """

    def __init__(self):
        super().__init__()
        self.title("Human Tracking System - Raw Control")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Config and hardware
        self.cfg = Config()
        # Make host-side rate-limiting permissive for immediacy
        self.cfg.set('arduino', 'min_command_interval_ms', 0.0)
        self.cfg.set('arduino', 'command_deadband_deg', 0.0)

        # Logger writes to the requested absolute path by default
        self.logger = RawSessionLogger()
        # VideoCapture expects a config dict
        self.video = VideoCapture({
            'capture_index': self.cfg.get('video', 'capture_index'),
            'width': self.cfg.get('video', 'width'),
            'height': self.cfg.get('video', 'height'),
            'fps': self.cfg.get('video', 'fps'),
            'buffer_size': 1,
        })
        # PoseDetector expects a full config dict
        self.detector = PoseDetector(self.cfg.get('pose_detection'))
        self.arduino = ArduinoController(self.cfg.get('arduino'))
        # Establish connection so immediate OutDelta commands move the motor
        try:
            self.arduino.connect()
        except Exception:
            pass

        # Raw mover and pipeline
        self.mover = RawMovementController(
            obArduino=self.arduino,
            fScaleDegPerUnit=float(self.cfg.get('pid', 'output_scale_deg') or 60.0),
            fDeadbandDeg=float(self.cfg.get('arduino', 'command_deadband_deg') or 0.0),
            logger=self.logger,
            fnOnSend=self._on_send,
        )
        self.pipe = RawHeadTrackingPipeline(self.detector, self.mover, self.logger)

        # UI elements
        self.lbl_offset = ctk.CTkLabel(self, text="Offset: 0.00", font=("Arial", 16))
        self.lbl_offset.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        self.slider = ctk.CTkSlider(self, from_=-30.0, to=30.0, width=300, command=self._on_slider)
        self.slider.grid(row=0, column=1, padx=10, pady=10)
        self.slider.set(0.0)

        self.lbl_canvas = ctk.CTkLabel(self, text="")
        self.lbl_canvas.grid(row=1, column=0, columnspan=2, padx=10, pady=10)

        self._last_slider = 0.0
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _on_send(self, fDeltaDeg: float):
        # Hook to observe sends; no-op for UI
        pass

    def _on_slider(self, fVal: float):
        # Immediate manual move by delta change
        fTarget = float(fVal)
        fDelta = float(fTarget - self._last_slider)
        self._last_slider = fTarget
        try:
            self.arduino.move_by_delta(fDelta)
            self.logger.log_info(f"CMD manual delta {fDelta:.2f}deg")
        except Exception as e:
            self.logger.log_failure(f"manual_move_error:{e}")

    def _loop(self):
        while self._running:
            ret, frame = self.video.get_frame(timeout=0.1)
            if ret and frame is not None:
                out = self.pipe.step(frame)
                # Update offset label
                if out.fErr is not None:
                    self.lbl_offset.configure(text=f"Offset: {out.fErr:.3f}")
                # Push frame to label
                img_rgb = cv2.cvtColor(out.obFrame if out.obFrame is not None else frame, cv2.COLOR_BGR2RGB)
                im = Image.fromarray(img_rgb)
                im_ctk = CTkImage(light_image=im, dark_image=im, size=(img_rgb.shape[1]//2, img_rgb.shape[0]//2))
                self.lbl_canvas.configure(image=im_ctk)
                self.lbl_canvas.image = im_ctk
            time.sleep(0.01)

    def on_close(self):
        self._running = False
        self.logger.close()
        self.destroy()


def run_app():
    app = SimpleTrackingApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()