from dataclasses import dataclass
from typing import Optional, Dict, Any
import time

try:
    import serial  # type: ignore
except Exception:
    serial = None  # Fallback to stub if pyserial is unavailable


@dataclass
class Feedback:
    current_angle: float
    current_speed: float
    is_running: bool
    target_angle: float
    timestamp_ms: int


class ArduinoController:
    """Hybrid Arduino controller.

    Tries to use serial protocol if available; otherwise falls back to a stub
    that emulates position locally so UI can run without hardware.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg or {}
        self.port = self.cfg.get('port', 'COM3')
        self.baud = int(self.cfg.get('baud', 115200))
        self._ser = None
        self.current_position: float = 0.0
        self._last_send_ms: int = 0
        self.min_interval_ms: float = float(self.cfg.get('min_command_interval_ms', 0.0) or 0.0)
        self.is_moving: bool = False

    def disconnect(self) -> None:
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            finally:
                self._ser = None

    def connect(self) -> bool:
        if serial is None:
            return False
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            return True
        except Exception:
            self._ser = None
            return False

    def move_by_delta(self, delta_deg: float) -> bool:
        # Rate limit if configured
        now_ms = int(time.time() * 1000)
        if self.min_interval_ms > 0 and (now_ms - self._last_send_ms) < self.min_interval_ms:
            return False
        self._last_send_ms = now_ms

        # If serial available, send absolute move using integrated target
        self.current_position += float(delta_deg)
        target_angle = self.current_position
        if self._ser:
            try:
                cmd = f"M,{target_angle}\n".encode('utf-8')
                self._ser.write(cmd)
                self.is_moving = True
                return True
            except Exception:
                return False
        else:
            # Stub mode: emulate movement immediately
            self.is_moving = abs(delta_deg) > 0.0
            return True

    def move_to_abs(self, angle_deg: float) -> bool:
        """Move to an absolute stage angle in degrees.

        Sends an 'M,<angle>' command if serial is available; otherwise updates
        the stubbed position directly. Clamps to [-180, 180] to match firmware.
        """
        # Clamp to stage limits
        target_angle = max(-180.0, min(180.0, float(angle_deg)))
        self.current_position = target_angle
        if self._ser:
            try:
                cmd = f"M,{target_angle}\n".encode('utf-8')
                self._ser.write(cmd)
                self.is_moving = True
                return True
            except Exception:
                return False
        else:
            # Stub mode: emulate movement immediately
            self.is_moving = True
            return True

    def get_feedback(self, timeout: float = 0.0) -> Optional[Feedback]:
        if self._ser:
            try:
                # Try to read a line; non-blocking behavior approximated
                line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('FB:'):
                    # FB:currentAngle,targetAngle,speed,isRunning,timestamp
                    parts = line[3:].split(',')
                    if len(parts) >= 5:
                        cur = float(parts[0])
                        tgt = float(parts[1])
                        spd = float(parts[2])
                        run = parts[3].strip() == '1'
                        ts = int(float(parts[4]))
                        self.current_position = cur
                        self.is_moving = run
                        return Feedback(cur, spd, run, tgt, ts)
            except Exception:
                pass
        # Stub feedback
        return Feedback(
            current_angle=float(self.current_position),
            current_speed=0.0,
            is_running=bool(self.is_moving),
            target_angle=float(self.current_position),
            timestamp_ms=int(time.time() * 1000),
        )