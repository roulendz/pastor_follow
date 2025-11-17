from config import Config
from face_centering import (
    EventLogger,
    CoordinateMapper,
    SmoothingPipeline,
    MovementController,
    PerformanceMonitor,
    FaceCenteringPipeline,
    FaceDetectionResult,
)


class DummyDetector:
    def __init__(self, x=0.2, y=0.5):
        self.x = x
        self.y = y
    def detect(self, frame):
        return FaceDetectionResult(x=self.x, y=self.y, confidence=0.9, timestamp=0.0, annotated_frame=None)


class DummyPID:
    def __init__(self):
        self.setpoint = 0.5
        self.filtered_derivative = 0.0
    def update(self, measured, dt):
        return (self.setpoint - float(measured)) * 8.0


class DummyArduino:
    def __init__(self):
        self.connected = True
        self.sent = []
    def move_by_delta(self, d):
        self.sent.append(float(d))
        return True


def test_pipeline_flow_basic():
    cfg = Config()
    cfg.set('tracking', 'smoothing_enabled', True)
    cfg.set('tracking', 'smoothing_window', 5)
    cfg.set('tracking', 'smoothing_alpha', 0.5)
    cfg.set('tracking', 'center_deadband_norm', 0.03)
    cfg.set('arduino', 'command_deadband_deg', 0.2)
    cfg.set('arduino', 'min_command_interval_ms', 40.0)
    cfg.set('arduino', 'output_slew_rate_deg_per_sec', 25.0)

    logger = EventLogger(ui_logger=None)
    detector = DummyDetector(x=0.2, y=0.5)
    smoother = SmoothingPipeline(smoother=None, config=cfg)
    # SmoothingPipeline requires smoother, but for the test we simulate no smoothing
    smoother.smoother = type("Dummy", (), {"update": lambda self, xy: xy})()
    mapper = CoordinateMapper(cfg)
    mover = MovementController(DummyPID(), DummyArduino(), cfg)
    perf = PerformanceMonitor()

    pipe = FaceCenteringPipeline(detector, smoother, mapper, mover, perf, logger)
    out = pipe.step(frame=None)
    assert out.detected is True
    assert out.move.delta_deg != 0.0  # a command was computed
    assert out.error_norm is not None