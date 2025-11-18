from typing import Any, Dict


class Config:
    """Minimal config shim providing nested dict access via get/set.

    Defaults chosen to run out-of-the-box for slider control.
    """

    def __init__(self):
        self._cfg: Dict[str, Dict[str, Any]] = {
            'video': {
                'capture_index': 0,
                'width': 1280,
                'height': 720,
                'fps': 60,
                'backend': 'DSHOW',  # Windows: DSHOW preferred; fallback to MSMF
                'fourcc': 'MJPG',    # Many webcams support 60fps with MJPG
                'buffersize': 1,
            },
            'arduino': {
                'port': 'COM3',
                'baud': 115200,
                'min_command_interval_ms': 0.0,
                'command_deadband_deg': 0.0,
            },
            'pid': {
                'kp': 1.0,
                'ki': 0.0,
                'kd': 0.0,
                'i_limit': 180.0,
                'output_scale_deg': 60.0,
            },
            'motion': {
                'max_speed_deg_s': 60.0,
                'curve_exponent': 1.6,
                'deadzone_norm': 0.05,
                'ease_tau_s': 0.2,
                'max_accel_deg_s2': 240.0,
                'auto_return_enabled': True,
                'return_idle_ms': 120,
                'return_speed_deg_s': 40.0,
                'min_command_deg': 0.02,
            },
            'tracking': {
                'slow_threshold_deg_s': 1.0,
                'engage_threshold_norm': 0.2,
                'unidir_hold_ms': 250,
            },
            'control': {
                'range_deg': 180,     # or 360
                'center_deg': 90,     # 180-range -> 90, 360-range -> 180
                'home_anim_ms': 250,  # smooth animation duration back to center
            },
        }

    def get(self, section: str, key: str = None):
        sec = self._cfg.get(section, {})
        if key is None:
            return sec
        return sec.get(key)

    def set(self, section: str, key: str, value: Any) -> None:
        self._cfg.setdefault(section, {})[key] = value