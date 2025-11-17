from __future__ import annotations
import time
from typing import Optional

from .types import PerformanceStats, PipelineOutput, MoveCommand
from .errors import FaceDetectionError, CoordinateError, MovementError


class FaceCenteringPipeline:
    """End-to-end pipeline orchestrating detection → smoothing → coords → control.

    This class coordinates the modular components while keeping each isolated.
    """

    def __init__(self, detector, smoother, mapper, mover, perf_monitor, logger):
        self.detector = detector
        self.smoother = smoother
        self.mapper = mapper
        self.mover = mover
        self.perf = perf_monitor
        self.log = logger
        self._last_time = time.time()

    def step(self, frame) -> PipelineOutput:
        t_total0 = time.time()
        stats = PerformanceStats()
        detection = None
        measured_x_raw = None
        measured_x_cal = None
        error_norm = None
        deadband_active = False
        move = MoveCommand(delta_deg=0.0, sent=False, reason="init")

        # Precise dt for control
        now = time.time()
        dt = max(1e-3, now - self._last_time)
        self._last_time = now

        # 1) Detect
        t0 = time.time()
        try:
            detection = self.detector.detect(frame)
            measured_x_raw = float(detection.x)
        except FaceDetectionError as e:
            self.log.error(f"detection error: {e}")
            return PipelineOutput(
                detected=False, detection=None, measured_x_raw=None, measured_x_cal=None,
                error_norm=None, deadband_active=False, move=move, perf=stats, debug={}
            )
        finally:
            stats.t_detect_ms = (time.time() - t0) * 1000.0

        # 2) Smooth
        t0 = time.time()
        try:
            sx, sy = self.smoother.update((detection.x, detection.y))
            measured_x_raw = float(sx)
        except Exception as e:
            # Smoothing errors are non-fatal; log and use raw value
            self.log.error(f"smoothing error: {e}")
            measured_x_raw = float(detection.x)
        finally:
            stats.t_smooth_ms = (time.time() - t0) * 1000.0

        # 3) Coordinates
        t0 = time.time()
        try:
            measured_x_cal, deadband_active = self.mapper.map_x(measured_x_raw)
            # Compute error using setpoint from mapper
            # mapper.map_x already used setpoint; recompute to expose externally
            try:
                setpoint = float(self.mapper.config.get('pid', 'setpoint') or 0.5)
            except Exception:
                setpoint = 0.5
            error_norm = setpoint - measured_x_cal
        except CoordinateError as e:
            self.log.error(f"coordinate error: {e}")
            return PipelineOutput(
                detected=True, detection=detection, measured_x_raw=measured_x_raw,
                measured_x_cal=None, error_norm=None, deadband_active=False,
                move=move, perf=stats, debug={}
            )
        finally:
            stats.t_coords_ms = (time.time() - t0) * 1000.0

        # 4) Control
        t0 = time.time()
        try:
            if deadband_active:
                # Damp derivative state if available to avoid ping-pong
                try:
                    self.mover.pid.filtered_derivative = 0.0
                except Exception:
                    pass
                move = MoveCommand(delta_deg=0.0, sent=False, reason="inside_deadband")
            else:
                delta_deg = self.mover.compute_delta(measured_x_cal, dt)
                move = self.mover.maybe_send(delta_deg)
        except MovementError as e:
            self.log.error(f"movement error: {e}")
            move = MoveCommand(delta_deg=0.0, sent=False, reason="movement_error")
        finally:
            stats.t_control_ms = (time.time() - t0) * 1000.0

        # Aggregation
        stats.t_total_ms = (time.time() - t_total0) * 1000.0
        self.perf.record(stats)

        debug = {
            "dt_ms": dt * 1000.0,
            "deadband_active": float(deadband_active),
            "move_sent": float(move.sent),
            "delta_deg": float(move.delta_deg),
        }

        return PipelineOutput(
            detected=True,
            detection=detection,
            measured_x_raw=measured_x_raw,
            measured_x_cal=measured_x_cal,
            error_norm=error_norm,
            deadband_active=deadband_active,
            move=move,
            perf=stats,
            debug=debug,
        )