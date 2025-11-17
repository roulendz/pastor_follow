"""
Human Tracking System - Main GUI Application
Integrates all modules for real-time human tracking with servo control
"""

import customtkinter as ctk
from customtkinter import CTkImage
import tkinter as tk
from tkinter import messagebox
import cv2
import numpy as np
from PIL import Image
import threading
import time
import queue
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import json
import os
import argparse

# Import our modules
from config import Config
from video_capture import VideoCapture, AsyncDeviceScanner
from pose_detection import PoseDetector, PositionSmoother
from pid_controller import AdaptivePIDController
from arduino_controller import ArduinoController, AsyncArduinoScanner

# Configure appearance
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class HumanTrackingApp(ctk.CTk):
    """Main application GUI"""
    
    def __init__(self, verbose: bool = False, log_file_path: str = None):
        super().__init__()
        # Verbose console echo flag
        self.verbose = bool(verbose)
        
        # Window setup
        self.title("Human Tracking System - Servo Control")
        self.geometry("1400x900")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Load configuration
        self.config = Config()
        
        # Initialize modules
        self.video_capture = VideoCapture(self.config.get('video'))
        self.pose_detector = PoseDetector(self.config.get('pose_detection'))
        self.position_smoother = PositionSmoother(
            window_size=self.config.get('tracking', 'smoothing_window'),
            alpha=self.config.get('pid', 'smoothing_factor')
        )
        self.pid_controller = AdaptivePIDController(
            kp=self.config.get('pid', 'kp'),
            ki=self.config.get('pid', 'ki'),
            kd=self.config.get('pid', 'kd'),
            setpoint=self.config.get('pid', 'setpoint'),
            output_limits=(self.config.get('pid', 'output_min'),
                          self.config.get('pid', 'output_max')),
            deadzone=self.config.get('pid', 'deadzone')
        )
        self.arduino = ArduinoController(self.config.get('arduino'))
        
        # Set callbacks
        self.arduino.set_feedback_callback(self.on_arduino_feedback)
        self.arduino.set_error_callback(self.on_arduino_error)
        
        # Control flags
        self.tracking_enabled = False
        self.running = True
        self.recording = False

        # Manual target selection
        self.manual_target_enabled = False
        self.manual_target_pos = None  # normalized (x, y)
        self.current_frame_size = (0, 0)  # (width, height) of displayed frame

        # Session log file (overwrite every session)
        # Write into the project directory regardless of current working directory
        app_dir = os.path.dirname(os.path.abspath(__file__))
        self.session_log_path = (
            log_file_path if isinstance(log_file_path, str) and len(log_file_path) > 0
            else os.path.join(app_dir, "last_session.log")
        )
        try:
            self.session_log_file = open(self.session_log_path, 'w', encoding='utf-8')
            # Initial log note
            self.log(f"Session log started: {self.session_log_path}")
        except Exception as e:
            self.session_log_file = None
            # Surface the error in the GUI log so it's visible
            try:
                self.log(f"Failed to open session log file: {e}")
            except Exception:
                pass
        
        # Queues for thread communication
        self.frame_display_queue = queue.Queue(maxsize=2)
        self.log_queue = queue.Queue(maxsize=100)
        
        # Performance metrics
        self.fps_history = []
        self.error_history = []
        self.position_history = []
        self.target_history = []
        self.output_history = []
        
        # Setup GUI
        self.setup_gui()
        
        # Start processing threads
        self.start_processing_threads()
        
        # Start GUI update loop
        self.update_gui()
    
    def setup_gui(self):
        """Setup the GUI layout"""
        # Create main container
        self.main_container = ctk.CTkFrame(self)
        self.main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Left panel - Video and controls
        self.left_panel = ctk.CTkFrame(self.main_container)
        self.left_panel.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        # Video display
        self.video_label = ctk.CTkLabel(self.left_panel, text="Video Feed")
        self.video_label.pack(padx=10, pady=10)
        # Click bindings for manual target control
        self.video_label.bind("<Button-1>", self.on_video_click)
        self.video_label.bind("<Button-3>", self.on_video_right_click)
        
        # Control buttons
        self.control_frame = ctk.CTkFrame(self.left_panel)
        self.control_frame.pack(fill="x", padx=10, pady=5)
        
        self.start_tracking_btn = ctk.CTkButton(
            self.control_frame, text="Start Tracking",
            command=self.toggle_tracking, width=120
        )
        self.start_tracking_btn.pack(side="left", padx=5)
        
        self.home_btn = ctk.CTkButton(
            self.control_frame, text="Home",
            command=self.home_position, width=80
        )
        self.home_btn.pack(side="left", padx=5)
        
        self.stop_btn = ctk.CTkButton(
            self.control_frame, text="Emergency Stop",
            command=self.emergency_stop, width=120,
            fg_color="red", hover_color="darkred"
        )
        self.stop_btn.pack(side="left", padx=5)
        
        self.record_btn = ctk.CTkButton(
            self.control_frame, text="Record",
            command=self.toggle_recording, width=80
        )
        self.record_btn.pack(side="left", padx=5)
        
        # Device selection
        self.device_frame = ctk.CTkFrame(self.left_panel)
        self.device_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(self.device_frame, text="Video:").pack(side="left", padx=5)
        self.video_device_var = tk.StringVar()
        self.video_device_menu = ctk.CTkOptionMenu(
            self.device_frame, variable=self.video_device_var,
            values=["Scanning..."], command=self.on_video_device_change
        )
        self.video_device_menu.pack(side="left", padx=5)
        
        self.scan_video_btn = ctk.CTkButton(
            self.device_frame, text="Scan Cameras", width=100,
            command=self.scan_video_devices
        )
        self.scan_video_btn.pack(side="left", padx=5)
        
        ctk.CTkLabel(self.device_frame, text="Arduino:").pack(side="left", padx=5)
        self.arduino_port_var = tk.StringVar()
        self.arduino_port_menu = ctk.CTkOptionMenu(
            self.device_frame, variable=self.arduino_port_var,
            values=["Scanning..."], command=self.on_arduino_port_change
        )
        self.arduino_port_menu.pack(side="left", padx=5)
        
        self.scan_arduino_btn = ctk.CTkButton(
            self.device_frame, text="Scan Arduinos", width=110,
            command=self.scan_arduino_devices
        )
        self.scan_arduino_btn.pack(side="left", padx=5)

        self.connect_arduino_btn = ctk.CTkButton(
            self.device_frame, text="Connect Arduino", width=130,
            command=self.connect_arduino
        )
        self.connect_arduino_btn.pack(side="left", padx=5)
        
        # Right panel - Settings and monitoring
        self.right_panel = ctk.CTkFrame(self.main_container, width=400)
        self.right_panel.pack(side="right", fill="both", padx=(5, 0))
        self.right_panel.pack_propagate(False)
        
        # Tabview for different settings
        self.tabview = ctk.CTkTabview(self.right_panel)
        self.tabview.pack(fill="both", expand=True, padx=10, pady=10)
        
        # PID Settings Tab
        self.pid_tab = self.tabview.add("PID Control")
        self.setup_pid_tab()
        
        # Tracking Settings Tab
        self.tracking_tab = self.tabview.add("Tracking")
        self.setup_tracking_tab()
        
        # Monitor Tab
        self.monitor_tab = self.tabview.add("Monitor")
        self.setup_monitor_tab()
        
        # Log Tab
        self.log_tab = self.tabview.add("Logs")
        self.setup_log_tab()
        
        # Status bar
        self.status_frame = ctk.CTkFrame(self)
        self.status_frame.pack(fill="x", padx=10, pady=(0, 10))
        
        self.status_label = ctk.CTkLabel(
            self.status_frame, text="Status: Ready", anchor="w"
        )
        self.status_label.pack(side="left", padx=10)
        
        self.fps_label = ctk.CTkLabel(
            self.status_frame, text="FPS: 0", anchor="e"
        )
        self.fps_label.pack(side="right", padx=10)
        
        # Start device scanning
        self.scan_video_devices()
        self.scan_arduino_devices()
    
    def setup_pid_tab(self):
        """Setup PID control settings tab"""
        # PID Gains
        gains_frame = ctk.CTkFrame(self.pid_tab)
        gains_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(gains_frame, text="PID Gains", font=("Arial", 14, "bold")).pack()
        
        # P Gain
        p_frame = ctk.CTkFrame(gains_frame)
        p_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(p_frame, text="P:", width=30).pack(side="left", padx=5)
        self.p_slider = ctk.CTkSlider(
            p_frame, from_=0, to=10, number_of_steps=100,
            command=self.on_pid_change
        )
        self.p_slider.set(self.config.get('pid', 'kp'))
        self.p_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.p_value = ctk.CTkLabel(p_frame, text=f"{self.config.get('pid', 'kp'):.2f}", width=50)
        self.p_value.pack(side="left", padx=5)
        
        # I Gain
        i_frame = ctk.CTkFrame(gains_frame)
        i_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(i_frame, text="I:", width=30).pack(side="left", padx=5)
        self.i_slider = ctk.CTkSlider(
            i_frame, from_=0, to=2, number_of_steps=100,
            command=self.on_pid_change
        )
        self.i_slider.set(self.config.get('pid', 'ki'))
        self.i_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.i_value = ctk.CTkLabel(i_frame, text=f"{self.config.get('pid', 'ki'):.3f}", width=50)
        self.i_value.pack(side="left", padx=5)
        
        # D Gain
        d_frame = ctk.CTkFrame(gains_frame)
        d_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(d_frame, text="D:", width=30).pack(side="left", padx=5)
        self.d_slider = ctk.CTkSlider(
            d_frame, from_=0, to=5, number_of_steps=100,
            command=self.on_pid_change
        )
        self.d_slider.set(self.config.get('pid', 'kd'))
        self.d_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.d_value = ctk.CTkLabel(d_frame, text=f"{self.config.get('pid', 'kd'):.2f}", width=50)
        self.d_value.pack(side="left", padx=5)
        
        # Motor Settings
        motor_frame = ctk.CTkFrame(self.pid_tab)
        motor_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(motor_frame, text="Motor Settings", font=("Arial", 14, "bold")).pack()
        
        # Max Speed
        speed_frame = ctk.CTkFrame(motor_frame)
        speed_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(speed_frame, text="Max Speed:", width=100).pack(side="left", padx=5)
        self.speed_entry = ctk.CTkEntry(speed_frame, width=100)
        self.speed_entry.insert(0, str(self.config.get('arduino', 'max_speed')))
        self.speed_entry.pack(side="left", padx=5)
        
        # Max Acceleration
        accel_frame = ctk.CTkFrame(motor_frame)
        accel_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(accel_frame, text="Max Accel:", width=100).pack(side="left", padx=5)
        self.accel_entry = ctk.CTkEntry(accel_frame, width=100)
        self.accel_entry.insert(0, str(self.config.get('arduino', 'max_acceleration')))
        self.accel_entry.pack(side="left", padx=5)
        
        # Apply button
        self.apply_motor_btn = ctk.CTkButton(
            motor_frame, text="Apply Motor Settings",
            command=self.apply_motor_settings
        )
        self.apply_motor_btn.pack(pady=10)

        # Invert direction checkbox
        try:
            invert_default = bool(self.config.get('arduino', 'invert_direction'))
        except Exception:
            invert_default = False
        self.invert_dir_var = tk.BooleanVar(value=invert_default)
        self.invert_dir_check = ctk.CTkCheckBox(
            motor_frame, text="Invert Direction", variable=self.invert_dir_var,
            command=self.on_invert_direction_change
        )
        self.invert_dir_check.pack(pady=5)

        # Command throttling (Deadband & Min Interval)
        throttle_frame = ctk.CTkFrame(self.pid_tab)
        throttle_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(throttle_frame, text="Command Throttling", font=("Arial", 14, "bold")).pack()

        # Deadband slider
        db_frame = ctk.CTkFrame(throttle_frame)
        db_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(db_frame, text="Deadband (deg):", width=140).pack(side="left", padx=5)
        self.deadband_slider = ctk.CTkSlider(
            db_frame, from_=0.0, to=0.50, number_of_steps=50,
            command=self.on_deadband_change
        )
        self.deadband_slider.set(self.config.get('arduino', 'command_deadband_deg'))
        self.deadband_value = ctk.CTkLabel(db_frame, text=f"{self.config.get('arduino', 'command_deadband_deg'):.3f}")
        self.deadband_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.deadband_value.pack(side="left", padx=5)

        # Min command interval slider
        mi_frame = ctk.CTkFrame(throttle_frame)
        mi_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(mi_frame, text="Min Cmd Interval (ms):", width=180).pack(side="left", padx=5)
        self.min_interval_slider = ctk.CTkSlider(
            mi_frame, from_=10, to=100, number_of_steps=90,
            command=self.on_min_interval_change
        )
        self.min_interval_slider.set(self.config.get('arduino', 'min_command_interval_ms'))
        self.min_interval_value = ctk.CTkLabel(mi_frame, text=f"{self.config.get('arduino', 'min_command_interval_ms'):.0f}")
        self.min_interval_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.min_interval_value.pack(side="left", padx=5)
        
        # Auto-tune options
        autotune_frame = ctk.CTkFrame(self.pid_tab)
        autotune_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(autotune_frame, text="Auto-Tuning", font=("Arial", 14, "bold")).pack()
        
        self.adaptive_var = tk.BooleanVar(value=False)
        self.adaptive_check = ctk.CTkCheckBox(
            autotune_frame, text="Enable Adaptive PID",
            variable=self.adaptive_var,
            command=self.toggle_adaptive_pid
        )
        self.adaptive_check.pack(pady=5)
    
    def setup_tracking_tab(self):
        """Setup tracking settings tab"""
        # Tracking mode
        mode_frame = ctk.CTkFrame(self.tracking_tab)
        mode_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(mode_frame, text="Tracking Mode", font=("Arial", 14, "bold")).pack()
        
        self.tracking_mode_var = tk.StringVar(value=self.config.get('tracking', 'tracking_mode'))
        modes = ["center", "largest", "closest"]
        for mode in modes:
            ctk.CTkRadioButton(
                mode_frame, text=mode.capitalize(),
                variable=self.tracking_mode_var, value=mode,
                command=self.on_tracking_mode_change
            ).pack(pady=2)
        
        # Target body part
        part_frame = ctk.CTkFrame(self.tracking_tab)
        part_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(part_frame, text="Target Body Part", font=("Arial", 14, "bold")).pack()
        
        self.body_part_var = tk.StringVar(value=self.config.get('tracking', 'target_body_part'))
        self.body_part_menu = ctk.CTkOptionMenu(
            part_frame, variable=self.body_part_var,
            values=list(PoseDetector.POSE_LANDMARKS.keys()),
            command=self.on_body_part_change
        )
        self.body_part_menu.pack(pady=5)
        
        # Smoothing settings
        smooth_frame = ctk.CTkFrame(self.tracking_tab)
        smooth_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(smooth_frame, text="Smoothing", font=("Arial", 14, "bold")).pack()
        
        self.smoothing_var = tk.BooleanVar(value=self.config.get('tracking', 'smoothing_enabled'))
        self.smoothing_check = ctk.CTkCheckBox(
            smooth_frame, text="Enable Smoothing",
            variable=self.smoothing_var,
            command=self.on_smoothing_change
        )
        self.smoothing_check.pack(pady=5)
        
        # Confidence threshold
        conf_frame = ctk.CTkFrame(self.tracking_tab)
        conf_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(conf_frame, text="Min Confidence:", width=100).pack(side="left", padx=5)
        self.confidence_slider = ctk.CTkSlider(
            conf_frame, from_=0, to=1, number_of_steps=20,
            command=self.on_confidence_change
        )
        self.confidence_slider.set(self.config.get('pose_detection', 'min_detection_confidence'))
        self.confidence_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.conf_value = ctk.CTkLabel(conf_frame, text="0.5", width=50)
        self.conf_value.pack(side="left", padx=5)
    
    def setup_monitor_tab(self):
        """Setup monitoring graphs tab"""
        # Create figure for plots
        self.fig = Figure(figsize=(5, 6), dpi=80)
        self.fig.subplots_adjust(hspace=0.4)
        
        # Error plot
        self.error_ax = self.fig.add_subplot(311)
        self.error_ax.set_title("Tracking Error")
        self.error_ax.set_ylabel("Error (normalized)")
        self.error_ax.set_ylim(-0.5, 0.5)
        self.error_line, = self.error_ax.plot([], [], 'b-')
        self.error_ax.grid(True, alpha=0.3)
        
        # PID Output plot
        self.output_ax = self.fig.add_subplot(312)
        self.output_ax.set_title("PID Output")
        self.output_ax.set_ylabel("Angle (degrees)")
        self.output_ax.set_ylim(-45, 45)
        self.output_line, = self.output_ax.plot([], [], 'r-')
        self.output_ax.grid(True, alpha=0.3)
        
        # Position plot
        self.position_ax = self.fig.add_subplot(313)
        self.position_ax.set_title("Motor Position")
        self.position_ax.set_ylabel("Angle (degrees)")
        self.position_ax.set_xlabel("Time (samples)")
        self.position_ax.set_ylim(-180, 180)
        self.target_line, = self.position_ax.plot([], [], 'g--', label='Target')
        self.actual_line, = self.position_ax.plot([], [], 'b-', label='Actual')
        self.position_ax.legend()
        self.position_ax.grid(True, alpha=0.3)
        
        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.monitor_tab)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)
        
        # Stats display
        self.stats_frame = ctk.CTkFrame(self.monitor_tab)
        self.stats_frame.pack(fill="x", padx=10, pady=5)
        
        self.stats_text = ctk.CTkLabel(
            self.stats_frame, text="Waiting for data...",
            justify="left", anchor="w"
        )
        self.stats_text.pack(padx=10, pady=5)
    
    def setup_log_tab(self):
        """Setup logging tab"""
        # Log text widget
        self.log_text = ctk.CTkTextbox(self.log_tab, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Log controls
        log_controls = ctk.CTkFrame(self.log_tab)
        log_controls.pack(fill="x", padx=10, pady=5)
        
        self.clear_log_btn = ctk.CTkButton(
            log_controls, text="Clear Log",
            command=lambda: self.log_text.delete("1.0", tk.END),
            width=100
        )
        self.clear_log_btn.pack(side="left", padx=5)
        
        self.save_log_btn = ctk.CTkButton(
            log_controls, text="Save Log",
            command=self.save_log,
            width=100
        )
        self.save_log_btn.pack(side="left", padx=5)
    
    def start_processing_threads(self):
        """Start background processing threads"""
        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()
        # Track last send time for performance metrics
        self.last_move_send_time = None
    
    def processing_loop(self):
        """Main processing loop running in background thread"""
        while self.running:
            try:
                # Get frame from video capture
                ret, frame = self.video_capture.get_frame(timeout=0.1)
                
                if ret and frame is not None:
                    # Process with pose detection (guard against errors)
                    try:
                        persons, annotated_frame = self.pose_detector.process_frame(frame)
                    except Exception as e:
                        annotated_frame = frame
                        persons = []
                        self.log(f"Pose processing error: {e}")
                    
                    # Get target position if tracking is enabled
                    if self.tracking_enabled:
                        target_pos = None

                        # Manual override: use clicked target if enabled
                        if self.manual_target_enabled and self.manual_target_pos is not None:
                            target_pos = (float(self.manual_target_pos[0]), float(self.manual_target_pos[1]))
                            # Draw marker at manual target
                            try:
                                mx = int(target_pos[0] * annotated_frame.shape[1])
                                my = int(target_pos[1] * annotated_frame.shape[0])
                                cv2.drawMarker(annotated_frame, (mx, my), (0, 255, 0),
                                               markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
                            except Exception:
                                pass
                        elif persons:
                            target_pos = self.pose_detector.get_target_position(
                                persons,
                                self.body_part_var.get(),
                                self.tracking_mode_var.get()
                            )
                            
                        if target_pos:
                            # Ensure numeric tuple (x, y)
                            try:
                                target_pos = (float(target_pos[0]), float(target_pos[1]))
                            except Exception:
                                # Skip this frame if target position is invalid
                                raise ValueError(f"Invalid target_pos: {target_pos}")

                            # Apply smoothing
                            if self.smoothing_var.get():
                                try:
                                    smoothed = self.position_smoother.update(target_pos)
                                    if smoothed is not None:
                                        target_pos = (float(smoothed[0]), float(smoothed[1]))
                                except Exception as e:
                                    # Fallback to raw target on smoothing errors
                                    self.log(f"Smoothing error: {e}")
                            
                            # Update PID controller with measured X (setpoint is 0.5)
                            measured_x = float(target_pos[0])
                            output = self.pid_controller.update(measured_x)
                            
                            # Send to Arduino
                            if self.arduino.connected:
                                if self.arduino.move_to_angle(output):
                                    # Record send time for timing analysis
                                    self.last_move_send_time = time.time()
                            
                            # Store for monitoring (error relative to center)
                            self.error_history.append(self.pid_controller.setpoint - measured_x)
                            self.output_history.append(output)
                            
                            # Draw tracking info on frame
                            cv2.putText(annotated_frame, 
                                    f"X: {measured_x:.3f} | Out: {output:.1f}°",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 255, 255), 2)
                    
                    # Add frame to display queue
                    try:
                        if self.frame_display_queue.full():
                            self.frame_display_queue.get_nowait()
                        self.frame_display_queue.put(annotated_frame)
                    except queue.Full:
                        pass
                
                # Small delay to prevent CPU spinning
                time.sleep(0.01)
                
            except Exception as e:
                self.log(f"Processing error: {e}")
                time.sleep(0.1)
    
    def update_gui(self):
        """Update GUI elements"""
        if not self.running:
            return
        
        # Update video display
        try:
            frame = self.frame_display_queue.get_nowait()
            self.display_frame(frame)
        except queue.Empty:
            pass
        
        # Update status
        stats = self.video_capture.get_stats()
        self.fps_label.configure(text=f"FPS: {stats.get('fps', 0):.1f}")
        
        # Compose clearer status including video and Arduino
        video_state = stats.get('state', 'STOPPED')
        status_text = f"Video: {video_state}"
        if self.arduino.connected:
            arduino_status = self.arduino.get_status()
            status_text += (
                f" | Arduino: Connected | Pos: {arduino_status['current_position']:.1f}° | "
                f"Target: {arduino_status['target_position']:.1f}° | "
                f"Cmds: {arduino_status['command_count']} | Fb: {arduino_status['feedback_count']}"
            )
        else:
            status_text += " | Arduino: Disconnected"
        self.status_label.configure(text=f"Status: {status_text}")
        
        # Update plots (every 5th frame to reduce CPU load)
        if hasattr(self, 'update_counter'):
            self.update_counter += 1
        else:
            self.update_counter = 0
        
        if self.update_counter % 5 == 0:
            self.update_plots()
        
        # Update logs
        self.update_logs()
        
        # Schedule next update
        self.after(33, self.update_gui)  # ~30 FPS
    
    def display_frame(self, frame):
        """Display frame in GUI"""
        # Resize frame to fit GUI
        height, width = frame.shape[:2]
        max_width = 800
        max_height = 600
        
        if width > max_width or height > max_height:
            scale = min(max_width/width, max_height/height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            frame = cv2.resize(frame, (new_width, new_height))
        
        # Convert to RGB and create image
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame_rgb)
        photo = CTkImage(image, size=(frame.shape[1], frame.shape[0]))
        
        # Track current displayed size for click normalization
        self.current_frame_size = (frame.shape[1], frame.shape[0])

        # Update label
        self.video_label.configure(image=photo, text="")
        self.video_label.image = photo  # Keep reference
    
    def update_plots(self):
        """Update monitoring plots"""
        if not self.error_history:
            return
        
        # Limit history length
        max_points = 100
        if len(self.error_history) > max_points:
            self.error_history = self.error_history[-max_points:]
            self.output_history = self.output_history[-max_points:]
        
        x = list(range(len(self.error_history)))
        
        # Update error plot
        self.error_line.set_data(x, self.error_history)
        self.error_ax.set_xlim(0, max(len(x), max_points))
        
        # Update output plot
        if self.output_history:
            self.output_line.set_data(x, self.output_history)
            self.output_ax.set_xlim(0, max(len(x), max_points))
        
        # Update position plot if Arduino connected
        if self.arduino.connected and self.arduino.last_feedback:
            if not hasattr(self, 'position_history'):
                self.position_history = []
                self.target_history = []
            
            self.position_history.append(self.arduino.last_feedback.current_angle)
            self.target_history.append(self.arduino.last_feedback.target_angle)
            
            if len(self.position_history) > max_points:
                self.position_history = self.position_history[-max_points:]
                self.target_history = self.target_history[-max_points:]
            
            x_pos = list(range(len(self.position_history)))
            self.actual_line.set_data(x_pos, self.position_history)
            self.target_line.set_data(x_pos, self.target_history)
            self.position_ax.set_xlim(0, max(len(x_pos), max_points))
        
        # Redraw canvas
        self.canvas.draw_idle()
        
        # Update stats text
        if self.pid_controller:
            metrics = self.pid_controller.get_performance_metrics()
            stats_text = f"Mean Error: {metrics.get('mean_error', 0):.3f}\n"
            stats_text += f"Settling Time: {metrics.get('settling_time', -1):.2f}s\n"
            stats_text += f"Overshoot: {metrics.get('overshoot', 0):.1f}%"
            self.stats_text.configure(text=stats_text)
    
    def update_logs(self):
        """Update log display"""
        try:
            while not self.log_queue.empty():
                msg = self.log_queue.get_nowait()
                timestamp = datetime.now().strftime("%H:%M:%S")
                self.log_text.insert(tk.END, f"[{timestamp}] {msg}\n")
                self.log_text.see(tk.END)
        except queue.Empty:
            pass
    
    def toggle_tracking(self):
        """Toggle tracking on/off"""
        self.tracking_enabled = not self.tracking_enabled
        
        if self.tracking_enabled:
            self.start_tracking_btn.configure(text="Stop Tracking")
            self.log("Tracking enabled")
            
            # Start video if not running
            if self.video_capture.state.name != 'RUNNING':
                device_str = self.video_device_var.get()
                if device_str and device_str != "Scanning...":
                    try:
                        device_idx = int(device_str.split()[1])
                        # Parse backend from option text e.g. "Device 0 (DirectShow)"
                        backend_name = None
                        lpar = device_str.find('(')
                        rpar = device_str.find(')')
                        if lpar != -1 and rpar != -1 and rpar > lpar:
                            backend_name = device_str[lpar+1:rpar]
                        # Map backend name to OpenCV constant
                        import cv2 as _cv
                        backend_map = {
                            'DirectShow': _cv.CAP_DSHOW,
                            'Media Foundation': _cv.CAP_MSMF,
                            'Any': _cv.CAP_ANY
                        }
                        backend_id = backend_map.get(backend_name, _cv.CAP_DSHOW)
                        self.video_capture.start(device_idx, backend=backend_id)
                    except:
                        self.log("Failed to parse device index")
        else:
            self.start_tracking_btn.configure(text="Start Tracking")
            self.log("Tracking disabled")
            self.position_smoother.reset()
            self.pid_controller.reset()
            # Clear recent histories to restore normal display
            self.error_history.clear()
            self.output_history.clear()
    
    def home_position(self):
        """Move to home position"""
        if self.arduino.connected:
            self.arduino.home()
            self.log("Moving to home position")
    
    def emergency_stop(self):
        """Emergency stop"""
        self.tracking_enabled = False
        self.start_tracking_btn.configure(text="Start Tracking")
        
        if self.arduino.connected:
            self.arduino.emergency_stop()
            self.log("Emergency stop activated")
    
    def toggle_recording(self):
        """Toggle video recording"""
        # Implementation for video recording
        self.recording = not self.recording
        if self.recording:
            self.record_btn.configure(text="Stop Rec", fg_color="red")
            self.log("Recording started")
        else:
            self.record_btn.configure(text="Record", fg_color=("gray75", "gray25"))
            self.log("Recording stopped")
    
    def scan_video_devices(self):
        """Scan for video devices asynchronously"""
        self.video_device_menu.configure(values=["Scanning..."])
        self.scan_video_btn.configure(state="disabled")
        
        def callback(devices):
            if devices:
                device_names = [f"Device {d.index} ({d.backend})" for d in devices]
                self.video_device_menu.configure(values=device_names)
                self.video_device_var.set(device_names[0])
            else:
                self.video_device_menu.configure(values=["No devices found"])
            self.scan_video_btn.configure(state="normal")
            self.log(f"Found {len(devices)} video devices")
        
        scanner = AsyncDeviceScanner(callback)
        scanner.scan_async()
    
    def scan_arduino_devices(self):
        """Scan for Arduino devices asynchronously"""
        self.arduino_port_menu.configure(values=["Scanning..."])
        self.scan_arduino_btn.configure(state="disabled")
        
        def callback(devices):
            if devices:
                port_names = [d.port for d in devices]
                self.arduino_port_menu.configure(values=port_names)
                self.arduino_port_var.set(port_names[0])
            else:
                self.arduino_port_menu.configure(values=["No devices found"])
            self.scan_arduino_btn.configure(state="normal")
            self.log(f"Found {len(devices)} Arduino devices")
        
        scanner = AsyncArduinoScanner(callback)
        scanner.scan_async()
    
    def on_video_device_change(self, value):
        """Handle video device selection change"""
        if value and value != "Scanning..." and value != "No devices found":
            try:
                device_idx = int(value.split()[1])
                # Parse backend name from option text
                backend_name = None
                lpar = value.find('(')
                rpar = value.find(')')
                if lpar != -1 and rpar != -1 and rpar > lpar:
                    backend_name = value[lpar+1:rpar]
                import cv2 as _cv
                backend_map = {
                    'DirectShow': _cv.CAP_DSHOW,
                    'Media Foundation': _cv.CAP_MSMF,
                    'Any': _cv.CAP_ANY
                }
                backend_id = backend_map.get(backend_name, _cv.CAP_DSHOW)
                if self.video_capture.state.name == 'RUNNING':
                    self.video_capture.stop()
                    time.sleep(0.5)
                    self.video_capture.start(device_idx, backend=backend_id)
                    self.log(f"Switched to video device {device_idx} ({backend_name or 'DirectShow'})")
            except:
                self.log("Failed to parse device index")
    
    def on_arduino_port_change(self, value):
        """Handle Arduino port selection change"""
        # Port selected, enable connect button
        if value and value != "Scanning..." and value != "No devices found":
            self.connect_arduino_btn.configure(state="normal")
    
    def connect_arduino(self):
        """Connect to selected Arduino"""
        port = self.arduino_port_var.get()
        if port and port != "Scanning..." and port != "No devices found":
            if self.arduino.connect(port):
                self.connect_arduino_btn.configure(text="Disconnect Arduino", fg_color="green")
                self.log(f"Connected to Arduino on {port}")
            else:
                self.log(f"Failed to connect to Arduino on {port}")
                messagebox.showerror("Connection Error", f"Failed to connect to Arduino on {port}")
        else:
            if self.arduino.connected:
                self.arduino.disconnect()
                self.connect_arduino_btn.configure(text="Connect Arduino", fg_color=("gray75", "gray25"))
                self.log("Disconnected from Arduino")
    
    def on_pid_change(self, value):
        """Handle PID slider changes"""
        kp = self.p_slider.get()
        ki = self.i_slider.get()
        kd = self.d_slider.get()
        
        self.p_value.configure(text=f"{kp:.2f}")
        self.i_value.configure(text=f"{ki:.3f}")
        self.d_value.configure(text=f"{kd:.2f}")
        
        self.pid_controller.set_tunings(kp, ki, kd)
        
        # Update config
        self.config.set('pid', 'kp', kp)
        self.config.set('pid', 'ki', ki)
        self.config.set('pid', 'kd', kd)
    
    def apply_motor_settings(self):
        """Apply motor settings to Arduino"""
        try:
            max_speed = float(self.speed_entry.get())
            max_accel = float(self.accel_entry.get())
            
            if self.arduino.connected:
                self.arduino.update_settings(
                    max_speed, max_accel,
                    self.pid_controller.kp,
                    self.pid_controller.ki,
                    self.pid_controller.kd
                )
                self.log("Motor settings applied")

            # Update config
            self.config.set('arduino', 'max_speed', max_speed)
            self.config.set('arduino', 'max_acceleration', max_accel)
        except ValueError:
            messagebox.showerror("Error", "Invalid motor settings values")

    def on_deadband_change(self, value):
        """Handle deadband slider changes"""
        try:
            db = float(value)
        except Exception:
            return
        self.deadband_value.configure(text=f"{db:.3f}")
        # Update controller and config immediately
        self.arduino.cmd_deadband_deg = db
        self.config.set('arduino', 'command_deadband_deg', db)
        self.log(f"Deadband set to {db:.3f}°")

    def on_min_interval_change(self, value):
        """Handle min command interval slider changes"""
        try:
            mi = float(value)
        except Exception:
            return
        self.min_interval_value.configure(text=f"{mi:.0f}")
        # Update controller and config immediately
        self.arduino.min_cmd_interval_ms = mi
        self.config.set('arduino', 'min_command_interval_ms', mi)
        self.log(f"Min command interval set to {mi:.0f} ms")
    
    def on_invert_direction_change(self):
        """Toggle motor direction polarity"""
        invert = bool(self.invert_dir_var.get())
        # Update config and controller immediately
        self.config.set('arduino', 'invert_direction', invert)
        try:
            self.arduino.set_direction_invert(invert)
        except Exception:
            # Fallback if controller not ready
            pass
        self.log(f"Invert direction {'ON' if invert else 'OFF'}")
    
    def toggle_adaptive_pid(self):
        """Toggle adaptive PID"""
        if self.adaptive_var.get():
            self.pid_controller.enable_adaptation(True)
            self.log("Adaptive PID enabled")
        else:
            self.pid_controller.enable_adaptation(False)
            self.log("Adaptive PID disabled")
    
    def on_tracking_mode_change(self):
        """Handle tracking mode change"""
        mode = self.tracking_mode_var.get()
        self.config.set('tracking', 'tracking_mode', mode)
        self.log(f"Tracking mode changed to: {mode}")
    
    def on_body_part_change(self, value):
        """Handle body part selection change"""
        self.config.set('tracking', 'target_body_part', value)
        self.log(f"Target body part changed to: {value}")
    
    def on_smoothing_change(self):
        """Handle smoothing toggle"""
        enabled = self.smoothing_var.get()
        self.config.set('tracking', 'smoothing_enabled', enabled)
        self.log(f"Smoothing {'enabled' if enabled else 'disabled'}")
    
    def on_confidence_change(self, value):
        """Handle confidence threshold change"""
        self.conf_value.configure(text=f"{value:.2f}")
        self.config.set('pose_detection', 'min_detection_confidence', value)
        self.pose_detector.pose.min_detection_confidence = value
    
    def on_arduino_feedback(self, feedback):
        """Handle Arduino feedback"""
        try:
            # Log both high-level objects and raw strings
            if isinstance(feedback, dict):
                # Not expected, but guard
                self.log(f"Arduino feedback (dict): {feedback}")
            elif hasattr(feedback, 'raw_data'):
                fb = feedback
                # Include response age since last send if available
                dt_ms = None
                try:
                    if self.last_move_send_time:
                        dt_ms = (time.time() - self.last_move_send_time) * 1000.0
                except Exception:
                    dt_ms = None
                if dt_ms is not None:
                    self.log(
                        f"FB current={fb.current_angle:.2f}° target={fb.target_angle:.2f}° "
                        f"speed={fb.current_speed:.1f} moving={int(fb.is_moving)} ts={fb.timestamp} dt_ms={dt_ms:.1f}"
                    )
                else:
                    self.log(
                        f"FB current={fb.current_angle:.2f}° target={fb.target_angle:.2f}° "
                        f"speed={fb.current_speed:.1f} moving={int(fb.is_moving)} ts={fb.timestamp}"
                    )
            elif isinstance(feedback, str):
                # Detect command sends, acks, and throttling info
                if feedback.startswith("CMD:"):
                    self.log(feedback)
                elif feedback.startswith("CMD_SKIP:"):
                    self.log(feedback)
                elif feedback.startswith("Move command acknowledged:"):
                    try:
                        dt_ms = None
                        if self.last_move_send_time:
                            dt_ms = (time.time() - self.last_move_send_time) * 1000.0
                        if dt_ms is not None:
                            self.log(f"{feedback} | ack_dt_ms={dt_ms:.1f}")
                        else:
                            self.log(feedback)
                    except Exception:
                        self.log(feedback)
                else:
                    self.log(feedback)
            else:
                self.log(f"Arduino feedback: {feedback}")
        except Exception as e:
            self.log(f"Feedback handling error: {e}")
    
    def on_arduino_error(self, error):
        """Handle Arduino error"""
        self.log(f"Arduino error: {error}")
    
    def log(self, message):
        """Add message to log queue"""
        try:
            self.log_queue.put_nowait(message)
        except queue.Full:
            pass
        # Also write to session log file immediately
        try:
            if getattr(self, 'session_log_file', None):
                ts = datetime.now().isoformat(timespec='seconds')
                self.session_log_file.write(f"[{ts}] {message}\n")
                self.session_log_file.flush()
            # Verbose console echo
            if self.verbose:
                ts = datetime.now().strftime("%H:%M:%S")
                print(f"[{ts}] {message}")
        except Exception:
            # Non-fatal; ignore file write errors to keep app running
            pass
    
    def save_log(self):
        """Save log to file"""
        from tkinter import filedialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                content = self.log_text.get("1.0", tk.END)
                with open(filename, 'w') as f:
                    f.write(content)
                self.log(f"Log saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save log: {e}")
    
    def on_closing(self):
        """Handle window closing"""
        self.running = False
        
        # Stop all modules
        if self.video_capture:
            self.video_capture.stop()
        if self.arduino.connected:
            self.arduino.disconnect()
        if self.pose_detector:
            self.pose_detector.cleanup()
        
        # Save configuration
        self.config.save_config()

        # Close session log file
        try:
            if getattr(self, 'session_log_file', None):
                self.session_log_file.close()
        except Exception:
            pass
        
        # Destroy window
        self.destroy()

    def on_video_click(self, event):
        """Handle left-click on video to set manual target (center on click)"""
        try:
            w, h = self.current_frame_size
            if w <= 0 or h <= 0:
                return
            x_norm = max(0.0, min(1.0, event.x / float(w)))
            y_norm = max(0.0, min(1.0, event.y / float(h)))
            self.manual_target_pos = (x_norm, y_norm)
            self.manual_target_enabled = True
            self.log(f"Manual target set by click: ({x_norm:.3f}, {y_norm:.3f})")
        except Exception as e:
            self.log(f"Video click error: {e}")

    def on_video_right_click(self, event):
        """Handle right-click to clear manual target"""
        self.manual_target_enabled = False
        self.manual_target_pos = None
        self.log("Manual target cleared")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Human Tracking System GUI")
    parser.add_argument("--verbose", action="store_true", help="Echo logs to console")
    parser.add_argument("--log-file", type=str, default=None, help="Path to session log file")
    args = parser.parse_args()

    app = HumanTrackingApp(verbose=args.verbose, log_file_path=args.log_file)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        # Gracefully close on Ctrl+C without a noisy traceback
        print("KeyboardInterrupt received; closing application...")
        try:
            app.on_closing()
        except Exception:
            pass
        try:
            app.destroy()
        except Exception:
            pass


if __name__ == "__main__":
    main()
