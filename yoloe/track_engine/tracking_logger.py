#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""有界异步 JSONL 日志：按大小轮转，慢磁盘不阻塞 tracking 主链路。"""
from __future__ import annotations

import json
import logging
import os
import queue
import threading
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any


class TrackingLogger:
    _instances = {}
    _lock = threading.Lock()

    def __init__(self, name: str, log_dir: str | None = None):
        self._name = name
        directory = Path(log_dir or os.getenv("TRACKING_LOG_DIR", "logs/tracking"))
        directory.mkdir(parents=True, exist_ok=True)
        # 固定进程名的轮转文件，重启服务也不会留下无限个时间戳文件。
        self._path = directory / f"{name}.jsonl"
        self._count = 0
        self.dropped = 0
        self.write_errors = 0
        self._counter_lock = threading.Lock()
        self.enabled = os.getenv("TRACKING_LOG_ENABLED", "1") != "0"
        self._queue = queue.Queue(maxsize=max(1, int(os.getenv("TRACKING_LOG_QUEUE_SIZE", "256"))))
        self._handler = RotatingFileHandler(
            self._path, maxBytes=max(1024, int(os.getenv("TRACKING_LOG_MAX_BYTES", "10485760"))),
            backupCount=max(1, int(os.getenv("TRACKING_LOG_BACKUPS", "3"))), encoding="utf-8",
        )
        self._handler.setFormatter(logging.Formatter("%(message)s"))
        self._worker = threading.Thread(target=self._write_loop, name=f"{name}-log", daemon=True)
        self._worker.start()
        print(f"[TRACKING_LOG] {name} -> {self._path}", flush=True)

    @classmethod
    def get(cls, name: str, log_dir: str | None = None):
        with cls._lock:
            if name not in cls._instances:
                cls._instances[name] = cls(name, log_dir)
            return cls._instances[name]

    @property
    def path(self):
        return self._path

    def log(self, data: dict[str, Any]):
        if not self.enabled:
            return
        with self._counter_lock:
            self._count += 1
            record = dict(_idx=self._count, _ts=time.time(), log_dropped=self.dropped,
                          log_write_errors=self.write_errors)
        record.update(data)
        line = json.dumps(record, ensure_ascii=False, default=str)
        try:
            self._queue.put_nowait(line)
        except queue.Full:
            with self._counter_lock:
                self.dropped += 1

    def _write_loop(self):
        while True:
            line = self._queue.get()
            try:
                # 直接执行轮转和写入，使磁盘异常可以计数而不是被 logging 静默吞掉。
                record = logging.LogRecord(self._name, logging.INFO, "", 0, line, (), None)
                if self._handler.shouldRollover(record):
                    self._handler.doRollover()
                self._handler.stream.write(line + "\n")
                self._handler.flush()
            except OSError as exc:
                self.write_errors += 1
                if self.write_errors == 1:
                    print(f"[TRACKING_LOG] write failed: {exc}", flush=True)
            finally:
                self._queue.task_done()

    def get_count(self):
        return self._count


# ── CUDA / GC 工具函数 ──

def cuda_memory_snapshot() -> dict[str, Any]:
    """采集当前 CUDA 内存状态（GPU 0）。"""
    snap: dict[str, Any] = {}
    try:
        import torch
        if torch.cuda.is_available():
            snap["cuda_allocated_mb"] = round(torch.cuda.memory_allocated(0) / 1024 / 1024, 1)
            snap["cuda_reserved_mb"] = round(torch.cuda.memory_reserved(0) / 1024 / 1024, 1)
            snap["cuda_max_allocated_mb"] = round(torch.cuda.max_memory_allocated(0) / 1024 / 1024, 1)
    except Exception:
        snap["cuda_error"] = "unavailable"
    return snap


def gc_snapshot() -> dict[str, Any]:
    """采集当前 Python GC 状态。"""
    import gc
    counts = gc.get_count()
    return {
        "gc_gen0": int(counts[0]),
        "gc_gen1": int(counts[1]),
        "gc_gen2": int(counts[2]),
        "gc_threshold": list(gc.get_threshold()),
        "gc_enabled": bool(gc.isenabled()),
    }
