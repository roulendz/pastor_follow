# Face Centering Architecture (SOLID, DRY)

This document describes the new modular architecture for the face-centering pipeline, designed to follow Single Responsibility Principle (SRP) and DRY.

## Component Diagram

```
┌────────────────────────────┐      ┌────────────────────────────┐
│ MediaPipeFaceDetectorAdapter│──►  │  SmoothingPipeline         │
│ (wraps PoseDetector)        │      │ (wraps PositionSmoother)  │
└──────────────┬──────────────┘      └──────────────┬──────────────┘
               │                                     │
               ▼                                     ▼
        ┌──────────────┐                      ┌──────────────┐
        │CoordinateMapper│  ─── error, db ───►│MovementController│──► Arduino
        │(calibration+DB)│                      │(PID, throttling)│
        └──────────────┬┘                      └──────────────┬┘
                       │                                     │
                       ▼                                     ▼
                ┌──────────────┐                     ┌─────────────────┐
                │PerformanceMonitor│◄── timings ────►│EventLogger       │
                └──────────────┘                     └─────────────────┘

                          ┌───────────────────────────────────────────┐
                          │        FaceCenteringPipeline               │
                          │ (orchestrates the sequence and error flow) │
                          └───────────────────────────────────────────┘
```

## Responsibilities & API Contracts

- `MediaPipeFaceDetectorAdapter`
  - Responsibility: isolate face target selection and error handling around `PoseDetector`.
  - API: `detect(frame) -> FaceDetectionResult`
  - Errors: raises `FaceDetectionError` on failures.

- `SmoothingPipeline`
  - Responsibility: runtime-togglable smoothing of `(x, y)` with `PositionSmoother`.
  - API: `update((x,y)) -> (sx, sy)`; respects `tracking.smoothing_enabled`.
  - Errors: non-fatal; logs errors and returns raw values.

- `CoordinateMapper`
  - Responsibility: apply inversion, offset, clamp; compute deadband vs setpoint.
  - API: `map_x(x) -> (x_calibrated, deadband_active)`
  - Errors: raises `CoordinateError` for invalid inputs/config.

- `MovementController`
  - Responsibility: compute PID-based delta with slew limit; enforce send deadband and min interval; send to Arduino.
  - API: `compute_delta(measured_x, dt) -> delta_deg`; `maybe_send(delta_deg) -> MoveCommand`
  - Errors: raises `MovementError` if Arduino send fails.

- `PerformanceMonitor`
  - Responsibility: collect timings per stage; summarize averages.
  - API: `record(PerformanceStats)`; `summary() -> dict`

- `EventLogger`
  - Responsibility: structured logs, optionally forwarded to GUI.
  - API: `info(msg)`, `debug(msg)`, `error(msg)`.

- `FaceCenteringPipeline`
  - Responsibility: orchestrate per-frame step: detection → smoothing → coords → control; collect timings, return `PipelineOutput`.
  - API: `step(frame) -> PipelineOutput`

## Usage Example

```python
from face_centering import (
    EventLogger, MediaPipeFaceDetectorAdapter, CoordinateMapper,
    SmoothingPipeline, MovementController, PerformanceMonitor,
    FaceCenteringPipeline
)
from pose_detection import PoseDetector, PositionSmoother
from pid_controller import AdaptivePIDController
from arduino_controller import ArduinoController
from config import Config

cfg = Config()
pose = PoseDetector(cfg)
smooth = PositionSmoother(window_size=int(cfg.get('tracking','smoothing_window') or 5),
                          alpha=float(cfg.get('tracking','smoothing_alpha') or 0.3))
pid = AdaptivePIDController()
arduino = ArduinoController(cfg)

logger = EventLogger()
detector = MediaPipeFaceDetectorAdapter(pose, cfg)
smoother = SmoothingPipeline(smooth, cfg)
mapper = CoordinateMapper(cfg)
mover = MovementController(pid, arduino, cfg)
perf = PerformanceMonitor()

pipeline = FaceCenteringPipeline(detector, smoother, mapper, mover, perf, logger)
output = pipeline.step(frame)  # one iteration
```

## Performance Characteristics

- Average timings are tracked per-stage and total; accessible via `PerformanceMonitor.summary()`.
- Slew rate, deadbands, and throttling reduce oscillations and chatter.
- Target: complete centering actions under 1 second with stable motion; tune via:
  - `pid` gains (`P`, `I`, `D`), `pid.setpoint`.
  - `arduino.output_slew_rate_deg_per_sec`, `arduino.min_command_interval_ms`, `arduino.command_deadband_deg`.
  - `tracking.smoothing_enabled`, `tracking.smoothing_window`, `tracking.smoothing_alpha`.

## Error Handling

- Detection errors short-circuit the step with `detected=False` and a default no-op `MoveCommand`.
- Coordinate and movement errors are logged and reported in `PipelineOutput.debug`; movement errors propagate as `MovementError`.