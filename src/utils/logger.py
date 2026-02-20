"""
src/utils/logger.py
Structured JSON-lines logger for the pipeline.
"""

from __future__ import annotations

import json
import logging
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any


class JsonLinesHandler(logging.Handler):
    """
    Writes structured JSON-lines records to a rotating log file.

    Each log call emits one JSON object per line:
      {"ts": <monotonic>, "level": "INFO", "module": "...", "msg": "...", ...extra}
    """

    def __init__(self, log_dir: str = "logs") -> None:
        super().__init__()
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = self._log_dir / f"run_{timestamp}.jsonl"
        self._file = open(self._log_path, "a", buffering=1)  # line-buffered

    def emit(self, record: logging.LogRecord) -> None:
        try:
            entry: dict[str, Any] = {
                "ts": time.monotonic(),
                "level": record.levelname,
                "module": record.name,
                "msg": record.getMessage(),
            }
            # Merge any extra kwargs attached as record attributes
            if hasattr(record, "_extra"):
                entry.update(record._extra)  # type: ignore[attr-defined]
            self._file.write(json.dumps(entry) + "\n")
        except Exception:
            self.handleError(record)

    def close(self) -> None:
        self._file.flush()
        self._file.close()
        super().close()

    @property
    def log_path(self) -> str:
        return str(self._log_path)


class StructuredLogger(logging.LoggerAdapter):
    """
    Logger adapter that accepts keyword arguments as structured fields.

    Usage::
        log.info("detection", cx=320.1, cy=240.5, conf=0.91)
    """

    def process(self, msg: str, kwargs: dict) -> tuple[str, dict]:
        extra_fields = kwargs.pop("extra_fields", {})
        # Pull any kwargs that are not standard logging kwargs into extra_fields
        standard_keys = {"exc_info", "stack_info", "stacklevel", "extra"}
        extra_fields.update({k: v for k, v in list(kwargs.items()) if k not in standard_keys})
        for k in list(extra_fields.keys()):
            kwargs.pop(k, None)

        if extra_fields:
            record_extra = kwargs.get("extra", {})
            record_extra["_extra"] = extra_fields
            kwargs["extra"] = record_extra

        return msg, kwargs


def get_logger(name: str, log_cfg: dict | None = None) -> StructuredLogger:
    """
    Build and return a StructuredLogger for *name*.

    Parameters
    ----------
    name : str
        Logger name (e.g. "pipeline", "detector").
    log_cfg : dict, optional
        Logging configuration with keys 'log_dir' and 'level'.
    """
    if log_cfg is None:
        log_cfg = {}

    log_dir = log_cfg.get("log_dir", "logs")
    level_name = log_cfg.get("level", "INFO")
    level = getattr(logging, level_name, logging.INFO)

    base_logger = logging.getLogger(name)
    base_logger.setLevel(level)

    # Avoid duplicate handlers if called multiple times
    if not any(isinstance(h, JsonLinesHandler) for h in base_logger.handlers):
        handler = JsonLinesHandler(log_dir)
        handler.setLevel(level)
        base_logger.addHandler(handler)

    return StructuredLogger(base_logger, {})
