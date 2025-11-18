import csv
import time
from datetime import datetime
from typing import Optional

try:
    import psutil  # optional
except Exception:
    psutil = None

import tracemalloc

from config import Config
from video_capture import VideoCapture


def get_proc_mem_mb() -> Optional[float]:
    try:
        if psutil:
            p = psutil.Process()
            return float(p.memory_info().rss) / (1024.0 * 1024.0)
    except Exception:
        pass
    try:
        # Fallback: tracemalloc gives Python allocations only
        snapshot = tracemalloc.take_snapshot()
        total = sum(stat.size for stat in snapshot.statistics('filename'))
        return float(total) / (1024.0 * 1024.0)
    except Exception:
        return None


def run_stress(duration_min: float = 10.0, csv_path: str = "video_stress_metrics.csv", min_fps_ok: float = 15.0):
    cfg = Config()
    vid_cfg = cfg.get('video')
    vc = VideoCapture({
        'capture_index': vid_cfg.get('capture_index'),
        'width': vid_cfg.get('width'),
        'height': vid_cfg.get('height'),
        'fps': vid_cfg.get('fps'),
        # accept both keys in VideoCapture; set explicitly here for clarity
        'buffersize': vid_cfg.get('buffersize', 1),
        'backend': vid_cfg.get('backend', 'DSHOW'),
        'fourcc': vid_cfg.get('fourcc', 'MJPG'),
    })

    tracemalloc.start()

    t_end = time.time() + (duration_min * 60.0)
    frame_total = 0
    last_print = 0.0

    with open(csv_path, mode='w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'timestamp', 'fps_measured', 'connected', 'backend', 'width', 'height',
            'fail_count', 'drops_1s', 'reinit_attempts', 'mem_mb'
        ])

        try:
            while time.time() < t_end:
                ok, frame = vc.get_frame(timeout=0.02)
                if ok and frame is not None:
                    frame_total += 1
                status = vc.get_status()
                fps_meas = float(status.get('fps_measured', 0.0))
                connected = bool(status.get('connected', False))
                backend = status.get('backend', '')
                width, height = status.get('resolution', (0, 0))
                fail_count = int(status.get('fail_count', 0))
                drops_1s = int(status.get('drops_1s', 0))
                reinit_attempts = int(status.get('reinit_attempts', 0))
                mem_mb = get_proc_mem_mb()

                w.writerow([
                    datetime.utcnow().isoformat(), f"{fps_meas:.2f}", int(connected), backend,
                    int(width), int(height), fail_count, drops_1s, reinit_attempts,
                    f"{mem_mb:.2f}" if mem_mb is not None else 'NA'
                ])
                f.flush()

                now = time.time()
                if (now - last_print) >= 1.0:
                    last_print = now
                    print(f"FPS {fps_meas:.1f} | backend={backend} connected={connected} drops_1s={drops_1s} reinit={reinit_attempts} mem={mem_mb:.1f if mem_mb else -1}MB")
                time.sleep(0.005)
        finally:
            vc.release()

    # Simple summary
    print(f"Total frames read: {frame_total}")
    print(f"CSV written: {csv_path}")
    print(f"Minimum acceptable FPS: {min_fps_ok}")


if __name__ == "__main__":
    # Default: 10 minutes stress test
    run_stress(duration_min=10.0)