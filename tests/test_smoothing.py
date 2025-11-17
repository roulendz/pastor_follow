from face_centering.smoothing import SmoothingPipeline
from pose_detection import PositionSmoother
from config import Config


def test_smoothing_pipeline_enabled():
    cfg = Config()
    cfg.set('tracking', 'smoothing_enabled', True)
    cfg.set('tracking', 'smoothing_window', 5)
    cfg.set('tracking', 'smoothing_alpha', 0.5)
    sp = SmoothingPipeline(PositionSmoother(window_size=5, alpha=0.5), cfg)
    out1 = sp.update((0.2, 0.5))
    out2 = sp.update((0.4, 0.5))
    assert out2[0] != 0.4  # smoothed


def test_smoothing_pipeline_disabled():
    cfg = Config()
    cfg.set('tracking', 'smoothing_enabled', False)
    sp = SmoothingPipeline(PositionSmoother(window_size=5, alpha=0.3), cfg)
    out = sp.update((0.3, 0.7))
    assert out == (0.3, 0.7)