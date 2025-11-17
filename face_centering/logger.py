from __future__ import annotations
import time
from typing import Optional


class EventLogger:
    """Lightweight structured logger that can also forward to a UI logger.

    This avoids coupling the pipeline to the GUI while still allowing rich logs.
    """

    def __init__(self, name: str = "face_centering", ui_logger: Optional[callable] = None, log_file_path: Optional[str] = None):
        self.name = name
        self.ui_logger = ui_logger
        self.log_file_path = log_file_path
        self._file = None
        if log_file_path:
            try:
                self._file = open(log_file_path, "a", encoding="utf-8")
            except Exception:
                self._file = None

    def _emit(self, level: str, msg: str):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {self.name} {level}: {msg}"
        if self.ui_logger:
            try:
                self.ui_logger(line)
            except Exception:
                pass
        if self._file:
            try:
                self._file.write(line + "\n")
                self._file.flush()
            except Exception:
                pass

    def info(self, msg: str):
        self._emit("INFO", msg)

    def debug(self, msg: str):
        self._emit("DEBUG", msg)

    def error(self, msg: str):
        self._emit("ERROR", msg)

    def close(self):
        if self._file:
            try:
                self._file.close()
            except Exception:
                pass
            self._file = None