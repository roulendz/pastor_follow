import pytest

from face_centering.coordinates import CoordinateMapper
from config import Config


def make_config(overrides: dict):
    cfg = Config()
    # apply overrides to known sections
    for (section, key), val in overrides.items():
        cfg.set(section, key, val)
    return cfg


def test_map_x_basic():
    cfg = make_config({
        ("calibration", "invert_horizontal"): False,
        ("calibration", "center_offset_x"): 0.0,
        ("tracking", "center_deadband_norm"): 0.05,
        ("pid", "setpoint"): 0.5,
    })
    mapper = CoordinateMapper(cfg)
    x_cal, db = mapper.map_x(0.47)
    assert pytest.approx(x_cal, 0.001) == 0.47
    assert db is True  # within deadband (< 0.05)


def test_map_x_invert_offset_clamp():
    cfg = make_config({
        ("calibration", "invert_horizontal"): True,
        ("calibration", "center_offset_x"): 0.1,
        ("calibration", "x_clamp_min"): 0.0,
        ("calibration", "x_clamp_max"): 1.0,
        ("tracking", "center_deadband_norm"): 0.02,
        ("pid", "setpoint"): 0.5,
    })
    mapper = CoordinateMapper(cfg)
    x_cal, db = mapper.map_x(0.2)
    # invert: 0.8 then +0.1 => 0.9
    assert pytest.approx(x_cal, 0.001) == 0.9
    assert db is False