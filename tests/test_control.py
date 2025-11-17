from face_centering.control import MovementController
from config import Config


class MockPID:
    def __init__(self):
        self.setpoint = 0.5
        self.filtered_derivative = 0.0
    def update(self, measured, dt):
        # simple proportional toward center
        return (self.setpoint - float(measured)) * 10.0


class MockArduino:
    def __init__(self):
        self.connected = True
        self.sent = []
    def move_by_delta(self, d):
        self.sent.append(float(d))
        return True


def test_movement_controller_throttling_and_deadband():
    cfg = Config()
    cfg.set('arduino', 'min_command_interval_ms', 50.0)
    cfg.set('arduino', 'command_deadband_deg', 0.5)
    cfg.set('arduino', 'output_slew_rate_deg_per_sec', 25.0)
    mover = MovementController(MockPID(), MockArduino(), cfg)

    # Large error → non-zero delta, should send
    d1 = mover.compute_delta(measured_x=0.0, dt=0.05)
    m1 = mover.maybe_send(d1)
    assert m1.sent is True

    # Immediately try again → throttled
    d2 = mover.compute_delta(measured_x=0.0, dt=0.01)
    m2 = mover.maybe_send(d2)
    assert m2.sent is False and m2.reason == 'throttled'

    # Small delta → below command deadband
    m3 = mover.maybe_send(0.1)
    assert m3.sent is False and m3.reason == 'below_cmd_deadband'