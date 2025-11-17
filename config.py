"""
Configuration file for Human Tracking System
"""

import json
import os

class Config:
    """Configuration management for the tracking system"""
    
    DEFAULT_CONFIG = {
        # Video settings
        "video": {
            "capture_index": 0,
            "width": 1280,
            "height": 720,
            "fps": 30,
            "buffer_size": 1
        },
        
        # MediaPipe settings
        "pose_detection": {
            "model_complexity": 1,  # 0, 1, or 2
            "min_detection_confidence": 0.5,
            "min_tracking_confidence": 0.5,
            "enable_segmentation": False,
            "smooth_landmarks": True,
            "face_only_bboxes": True,
            "min_visibility": 0.5
        },
        
        # PID Controller settings
        "pid": {
            "kp": 0.8,
            "ki": 0.05,
            "kd": 0.2,
            "setpoint": 0.5,  # Center of frame
            "output_min": -45,  # degrees
            "output_max": 45,   # degrees
            "deadzone": 0.02,   # 2% of frame width
            "smoothing_factor": 0.3
        },
        
        # Arduino settings
        "arduino": {
            "port": "COM3",
            "baudrate": 115200,
            "timeout": 0.1,
            "max_speed": 5000,
            "max_acceleration": 2000,
            "feedback_rate": 20,  # Hz
            "min_command_interval_ms": 40,
            "command_deadband_deg": 0.05,
            "output_slew_rate_deg_per_sec": 60,
            "max_deg_per_sec": 30,
            "steps_per_rev": 200,
            "gear_ratio": 180,
            "microsteps": 8
        },
        
        # Tracking settings
        "tracking": {
            "target_body_part": "nose",  # nose, left_shoulder, right_shoulder, etc.
            "tracking_mode": "center",   # center, largest, closest
            "smoothing_enabled": True,
            "smoothing_window": 5,
            "min_visibility": 0.5,
            "smoothing_alpha": 0.3,
            "focus_on_face": True
        },
        
        # GUI settings
        "gui": {
            "theme": "dark",
            "update_rate": 30,  # FPS for GUI updates
            "show_skeleton": True,
            "show_fps": True,
            "show_pid_output": True,
            "plot_history_length": 100
        },
        
        # Performance settings
        "performance": {
            "use_gpu": False,
            "max_frame_skip": 2,
            "processing_threads": 2
        }
    }
    
    def __init__(self, config_file="config.json"):
        self.config_file = config_file
        self.config = self.load_config()
    
    def load_config(self):
        """Load configuration from file or create default"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    loaded_config = json.load(f)
                    # Merge with default config to ensure all keys exist
                    return self.merge_configs(self.DEFAULT_CONFIG, loaded_config)
            except Exception as e:
                print(f"Error loading config: {e}")
                return self.DEFAULT_CONFIG.copy()
        else:
            self.save_config(self.DEFAULT_CONFIG)
            return self.DEFAULT_CONFIG.copy()
    
    def merge_configs(self, default, loaded):
        """Recursively merge loaded config with default"""
        merged = default.copy()
        for key, value in loaded.items():
            if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
                merged[key] = self.merge_configs(merged[key], value)
            else:
                merged[key] = value
        return merged
    
    def save_config(self, config=None):
        """Save configuration to file"""
        if config is None:
            config = self.config
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving config: {e}")
            return False
    
    def get(self, section, key=None):
        """Get configuration value"""
        if key is None:
            return self.config.get(section, {})
        return self.config.get(section, {}).get(key, None)
    
    def set(self, section, key, value):
        """Set configuration value"""
        if section not in self.config:
            self.config[section] = {}
        self.config[section][key] = value
        self.save_config()
    
    def update_section(self, section, values):
        """Update entire configuration section"""
        if section in self.config:
            self.config[section].update(values)
            self.save_config()
