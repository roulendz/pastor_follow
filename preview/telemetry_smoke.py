import time
import sys
from pathlib import Path

# Ensure repository root is on sys.path for imports
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from raw_tracking.slider_logger import SliderCSVLogger


def main():
    # Write a few sample rows to confirm CSV telemetry logging
    logger = SliderCSVLogger(
        log_path="f:\\Documents\\Arduino\\pastor_follow_v2\\last_session.log",
        write_header=False,
        slow_threshold_deg_s=1.0,
        engage_threshold_norm=0.2,
        unidir_hold_ms=250,
    )

    # Generate 12 samples with varying slider/speed to exercise flags
    for i in range(12):
        slider = (i - 6) / 6.0  # range roughly [-1, 1]
        angle = i * 1.5
        speed = 0.5 if i % 3 == 0 else (5.0 if i % 2 == 0 else -5.0)
        logger.log_sample(slider_norm=slider, motor_angle_deg=angle, motor_speed_deg_s=speed)
        time.sleep(0.03)

    logger.close()
    print("Telemetry smoke: wrote 12 samples")


if __name__ == "__main__":
    main()