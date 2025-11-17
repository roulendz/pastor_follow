"""
PID Controller Module
Implements a PID controller for smooth servo motor control
"""

import time
import numpy as np
from collections import deque
from typing import Optional, Tuple
import threading

class PIDController:
    """PID Controller with anti-windup and derivative filtering"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 setpoint: float = 0.0, output_limits: Tuple[float, float] = (-180, 180),
                 deadzone: float = 0.01, sample_time: float = 0.05):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Target value
            output_limits: (min, max) output limits
            deadzone: Error threshold below which output is zero
            sample_time: Expected time between updates (seconds)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.deadzone = deadzone
        self.sample_time = sample_time
        
        # State variables
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.last_output = 0
        
        # Derivative filter (low-pass)
        self.derivative_filter_coefficient = 0.1
        self.filtered_derivative = 0
        
        # Anti-windup
        self.integral_limit = 100  # Prevent integral from growing too large
        self.windup_guard = True
        
        # Performance tracking
        self.error_history = deque(maxlen=100)
        self.output_history = deque(maxlen=100)
        self.time_history = deque(maxlen=100)
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Auto-tuning parameters
        self.auto_tune_enabled = False
        self.auto_tune_data = []
    
    def update(self, measured_value: float, dt: Optional[float] = None) -> float:
        """
        Calculate PID output value for given measured value
        
        Args:
            measured_value: Current measured value
            dt: Time delta since last update (if None, calculated automatically)
        
        Returns:
            Control output value
        """
        with self.lock:
            current_time = time.time()
            
            if dt is None:
                dt = current_time - self.last_time
            
            if dt <= 0:
                return self.last_output
            
            # Calculate error
            error = self.setpoint - measured_value
            
            # Apply deadzone
            if abs(error) < self.deadzone:
                error = 0
            
            # Proportional term
            P = self.kp * error
            
            # Integral term with anti-windup
            self.integral += error * dt
            
            # Apply integral limits (anti-windup)
            if self.windup_guard:
                self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
            
            I = self.ki * self.integral
            
            # Derivative term with filtering
            if dt > 0:
                derivative = (error - self.last_error) / dt
                # Low-pass filter on derivative
                self.filtered_derivative = (
                    self.derivative_filter_coefficient * derivative +
                    (1 - self.derivative_filter_coefficient) * self.filtered_derivative
                )
            else:
                self.filtered_derivative = 0
            
            D = self.kd * self.filtered_derivative
            
            # Calculate total output
            output = P + I + D
            
            # Apply output limits
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
            
            # Back-calculation anti-windup
            if self.windup_guard and (output != P + I + D):
                # If output was saturated, prevent integral from growing
                excess = output - (P + I + D)
                self.integral -= excess * dt / self.ki if self.ki != 0 else 0
            
            # Store state for next iteration
            self.last_error = error
            self.last_time = current_time
            self.last_output = output
            
            # Track performance
            self.error_history.append(error)
            self.output_history.append(output)
            self.time_history.append(current_time)
            
            # Auto-tuning data collection
            if self.auto_tune_enabled:
                self.auto_tune_data.append({
                    'time': current_time,
                    'error': error,
                    'output': output,
                    'P': P,
                    'I': I,
                    'D': D
                })
            
            return output
    
    def set_tunings(self, kp: float, ki: float, kd: float):
        """Update PID tuning parameters"""
        with self.lock:
            self.kp = kp
            self.ki = ki
            self.kd = kd
    
    def set_setpoint(self, setpoint: float):
        """Update the setpoint"""
        with self.lock:
            self.setpoint = setpoint
    
    def reset(self):
        """Reset the PID controller state"""
        with self.lock:
            self.last_error = 0
            self.integral = 0
            self.filtered_derivative = 0
            self.last_time = time.time()
            self.last_output = 0
            self.error_history.clear()
            self.output_history.clear()
            self.time_history.clear()
    
    def get_state(self) -> dict:
        """Get current controller state"""
        with self.lock:
            return {
                'kp': self.kp,
                'ki': self.ki,
                'kd': self.kd,
                'setpoint': self.setpoint,
                'last_error': self.last_error,
                'integral': self.integral,
                'last_output': self.last_output,
                'filtered_derivative': self.filtered_derivative
            }
    
    def get_performance_metrics(self) -> dict:
        """Calculate performance metrics"""
        with self.lock:
            if len(self.error_history) < 2:
                return {}
            
            errors = np.array(self.error_history)
            outputs = np.array(self.output_history)
            
            return {
                'mean_error': np.mean(np.abs(errors)),
                'std_error': np.std(errors),
                'max_error': np.max(np.abs(errors)),
                'mean_output': np.mean(outputs),
                'std_output': np.std(outputs),
                'settling_time': self._calculate_settling_time(),
                'overshoot': self._calculate_overshoot()
            }
    
    def _calculate_settling_time(self, threshold: float = 0.02) -> float:
        """Calculate settling time (time to reach within threshold of setpoint)"""
        if len(self.error_history) < 10:
            return -1
        
        errors = np.array(self.error_history)
        threshold_value = threshold * abs(self.setpoint) if self.setpoint != 0 else threshold
        
        # Find last time error exceeded threshold
        settling_index = -1
        for i in range(len(errors) - 1, -1, -1):
            if abs(errors[i]) > threshold_value:
                settling_index = i
                break
        
        if settling_index == -1:
            return 0  # Already settled
        
        if settling_index == len(errors) - 1:
            return -1  # Not yet settled
        
        # Calculate time from start to settling
        return self.time_history[settling_index] - self.time_history[0]
    
    def _calculate_overshoot(self) -> float:
        """Calculate percentage overshoot"""
        if len(self.error_history) < 10:
            return 0
        
        if self.setpoint == 0:
            return 0
        
        errors = np.array(self.error_history)
        
        # Overshoot occurs when error crosses the setpoint and has opposite sign.
        # For positive setpoint, negative error indicates overshoot; magnitude is -min(error).
        # For negative setpoint, positive error indicates overshoot; magnitude is max(error).
        if self.setpoint > 0:
            overshoot_mag = max(0.0, float(-np.min(errors)))
        else:
            overshoot_mag = max(0.0, float(np.max(errors)))

        return (overshoot_mag / abs(self.setpoint)) * 100 if self.setpoint != 0 else 0
    
    def auto_tune(self, method: str = 'ziegler_nichols'):
        """Auto-tune PID parameters using specified method"""
        # Placeholder for auto-tuning implementation
        # This would require system identification and tuning algorithms
        pass


class AdaptivePIDController(PIDController):
    """Adaptive PID controller that adjusts gains based on performance"""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.adaptation_rate = 0.01
        self.performance_window = 50
        self.adapt_enabled = False
        self.target_settling_time = 1.0  # seconds
        self.target_overshoot = 10.0  # percent
    
    def update(self, measured_value: float, dt: Optional[float] = None) -> float:
        """Update with adaptive gain adjustment"""
        output = super().update(measured_value, dt)
        
        if self.adapt_enabled and len(self.error_history) >= self.performance_window:
            self._adapt_gains()
        
        return output
    
    def _adapt_gains(self):
        """Adapt PID gains based on performance metrics"""
        metrics = self.get_performance_metrics()
        
        if not metrics:
            return
        
        settling_time = metrics.get('settling_time', -1)
        overshoot = metrics.get('overshoot', 0)
        
        # Simple adaptation rules
        if settling_time > self.target_settling_time and settling_time > 0:
            # System is too slow, increase P gain
            self.kp *= (1 + self.adaptation_rate)
        
        if overshoot > self.target_overshoot:
            # System is overshooting, decrease P gain or increase D gain
            self.kp *= (1 - self.adaptation_rate * 0.5)
            self.kd *= (1 + self.adaptation_rate)
        
        # Adjust integral gain based on steady-state error
        mean_error = metrics.get('mean_error', 0)
        if mean_error > self.deadzone * 2:
            self.ki *= (1 + self.adaptation_rate * 0.5)
        
        # Apply limits to prevent instability
        self.kp = np.clip(self.kp, 0.1, 10.0)
        self.ki = np.clip(self.ki, 0, 1.0)
        self.kd = np.clip(self.kd, 0, 5.0)
    
    def enable_adaptation(self, enabled: bool = True):
        """Enable or disable adaptive tuning"""
        self.adapt_enabled = enabled


class MultiAxisPIDController:
    """Controller for multiple axes (e.g., pan and tilt)"""
    
    def __init__(self, num_axes: int = 2):
        self.controllers = []
        for i in range(num_axes):
            self.controllers.append(AdaptivePIDController())
        self.num_axes = num_axes
    
    def update(self, measured_values: list, dt: Optional[float] = None) -> list:
        """Update all controllers"""
        if len(measured_values) != self.num_axes:
            raise ValueError(f"Expected {self.num_axes} values, got {len(measured_values)}")
        
        outputs = []
        for controller, value in zip(self.controllers, measured_values):
            outputs.append(controller.update(value, dt))
        
        return outputs
    
    def set_setpoints(self, setpoints: list):
        """Set setpoints for all axes"""
        if len(setpoints) != self.num_axes:
            raise ValueError(f"Expected {self.num_axes} setpoints, got {len(setpoints)}")
        
        for controller, setpoint in zip(self.controllers, setpoints):
            controller.set_setpoint(setpoint)
    
    def reset_all(self):
        """Reset all controllers"""
        for controller in self.controllers:
            controller.reset()
