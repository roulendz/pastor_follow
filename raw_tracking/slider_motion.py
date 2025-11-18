import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class PIDConfig:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    i_limit: float = 180.0  # integral windup guard (degrees)


class PIDController:
    def __init__(self, cfg: PIDConfig):
        self.cfg = cfg
        self._i: float = 0.0
        self._prev_error: Optional[float] = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev_error = None

    def update(self, error_deg: float, dt_s: float) -> float:
        if dt_s <= 0:
            # Avoid division by zero; return proportional term only
            d = 0.0
        else:
            d = 0.0 if self._prev_error is None else (error_deg - self._prev_error) / dt_s

        # Integrator with clamping
        self._i += error_deg * dt_s
        if self._i > self.cfg.i_limit:
            self._i = self.cfg.i_limit
        elif self._i < -self.cfg.i_limit:
            self._i = -self.cfg.i_limit

        self._prev_error = error_deg

        return (
            self.cfg.kp * error_deg
            + self.cfg.ki * self._i
            + self.cfg.kd * d
        )


@dataclass
class MotionConfig:
    # Velocity mapping
    max_speed_deg_s: float = 60.0
    curve_exponent: float = 1.6  # >1 gives ease-in/out
    deadzone_norm: float = 0.05

    # Smoothing and limiting
    ease_tau_s: float = 0.2  # low-pass filter time constant for velocity
    max_accel_deg_s2: float = 240.0  # slew limit for velocity changes

    # Auto-return behavior
    auto_return_enabled: bool = True
    return_idle_ms: int = 120  # consider released if no input updates within this time
    return_speed_deg_s: float = 40.0  # max return speed

    # Safety
    min_command_deg: float = 0.02  # below this, do not send commands


class SliderMotionController:
    """Maps slider input [-1,1] to smooth motor movement.

    - Proportional speed to displacement with ease-in/out curve.
    - Deadzone around center.
    - Low-pass smoothing of commanded velocity and slew rate limiting.
    - Optional auto-return to center when input is idle.
    - PID closes the loop to hit the evolving target angle.
    """

    def __init__(self, cfg: MotionConfig, pid_cfg: PIDConfig):
        self.cfg = cfg
        self.pid = PIDController(pid_cfg)

        self._user_norm: float = 0.0
        self._last_user_ms: int = self._now_ms()
        self._target_angle_deg: float = 0.0  # evolving target angle
        self._vel_cmd_deg_s: float = 0.0     # smoothed, limited velocity command

    def _now_ms(self) -> int:
        return int(time.time() * 1000)

    def set_user_input(self, slider_norm: float) -> None:
        # Clamp input to [-1,1]
        if slider_norm > 1.0:
            slider_norm = 1.0
        elif slider_norm < -1.0:
            slider_norm = -1.0

        # Deadzone
        if abs(slider_norm) < self.cfg.deadzone_norm:
            slider_norm = 0.0

        self._user_norm = slider_norm
        self._last_user_ms = self._now_ms()

    @property
    def user_norm(self) -> float:
        return self._user_norm

    @property
    def target_angle_deg(self) -> float:
        return self._target_angle_deg

    @property
    def vel_cmd_deg_s(self) -> float:
        return self._vel_cmd_deg_s

    def _map_norm_to_speed(self, norm: float, max_speed: float) -> float:
        # Ease-in/out power curve, sign preserved
        magnitude = abs(norm)
        if magnitude <= 0.0:
            return 0.0
        return max_speed * (magnitude ** self.cfg.curve_exponent) * (1.0 if norm >= 0 else -1.0)

    def _low_pass(self, current: float, target: float, dt_s: float, tau_s: float) -> float:
        if tau_s <= 0.0:
            return target
        alpha = dt_s / (tau_s + dt_s)
        return current + alpha * (target - current)

    def update(self, dt_s: float, motor_angle_deg: float, motor_speed_deg_s: float) -> float:
        """Advance controller by dt.

        Returns the delta angle command (degrees) to send this cycle.
        """
        if dt_s <= 0.0:
            return 0.0

        now_ms = self._now_ms()
        idle_ms = now_ms - self._last_user_ms

        # Decide control input source: user vs auto-return
        if self.cfg.auto_return_enabled and idle_ms >= self.cfg.return_idle_ms:
            # Drive back toward center using return_speed with same easing
            # Direction is opposite of current target angle
            direction = -1.0 if self._target_angle_deg > 0 else (1.0 if self._target_angle_deg < 0 else 0.0)
            # Normalize the remaining offset to a synthetic norm in [0,1]
            # Larger offsets return faster; near center return is gentle due to smoothing
            synthetic_norm = min(1.0, abs(self._target_angle_deg) / (self.cfg.max_speed_deg_s))
            v_target = self._map_norm_to_speed(direction * synthetic_norm, self.cfg.return_speed_deg_s)
        else:
            v_target = self._map_norm_to_speed(self._user_norm, self.cfg.max_speed_deg_s)

        # Low-pass smooth commanded velocity (ease-in/out)
        v_smooth = self._low_pass(self._vel_cmd_deg_s, v_target, dt_s, self.cfg.ease_tau_s)

        # Slew-rate limit for safety
        dv = v_smooth - self._vel_cmd_deg_s
        dv_max = self.cfg.max_accel_deg_s2 * dt_s
        if dv > dv_max:
            v_smooth = self._vel_cmd_deg_s + dv_max
        elif dv < -dv_max:
            v_smooth = self._vel_cmd_deg_s - dv_max

        self._vel_cmd_deg_s = v_smooth

        # Evolve target angle by integrating the smoothed velocity
        self._target_angle_deg += self._vel_cmd_deg_s * dt_s

        # PID close the loop on angle
        error_deg = self._target_angle_deg - motor_angle_deg
        pid_out_deg = self.pid.update(error_deg, dt_s)

        # Limit the instantaneous command to what current velocity allows
        max_step_this_cycle = max(abs(self._vel_cmd_deg_s) * dt_s, self.cfg.min_command_deg)
        if pid_out_deg > max_step_this_cycle:
            pid_out_deg = max_step_this_cycle
        elif pid_out_deg < -max_step_this_cycle:
            pid_out_deg = -max_step_this_cycle

        # Suppress tiny commands
        if abs(pid_out_deg) < self.cfg.min_command_deg:
            return 0.0

        return pid_out_deg


# Utility mapping helpers for DRY and unit tests
def map_deg_to_norm(deg: float, center_deg: float, range_deg: float) -> float:
    """Map an absolute degree value to normalized [-1,1] around center.

    Example: center=90, range=180 => 0..180 maps to -1..1.
    """
    half = max(1e-6, float(range_deg) / 2.0)
    norm = (float(deg) - float(center_deg)) / half
    if norm < -1.0:
        return -1.0
    if norm > 1.0:
        return 1.0
    return norm


def map_norm_to_deg(norm: float, center_deg: float, range_deg: float) -> float:
    """Inverse of map_deg_to_norm."""
    half = max(1e-6, float(range_deg) / 2.0)
    return float(center_deg) + float(norm) * half


def plan_center_animation_steps(current_deg: float, center_deg: float, duration_ms: int, step_ms: int = 16) -> list:
    """Generate degree steps for a smooth ease-out animation returning to center.

    Uses an ease-out cubic that guarantees the last step reaches center.
    """
    steps = []
    total = max(1, int(duration_ms // max(1, step_ms)))
    delta = float(center_deg - current_deg)
    for i in range(1, total + 1):
        t = float(i) / float(total)
        ease = 1.0 - (1.0 - t) ** 3  # ease-out cubic
        steps.append(float(current_deg + delta * ease))
    # Ensure exact center on the last step
    if steps:
        steps[-1] = float(center_deg)
    return steps