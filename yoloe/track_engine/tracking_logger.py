#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享的结构化 JSON 文件日志模块，供 YOLOE tracker 服务端和 agent 客户端使用。"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Any


class TrackingLogger:
    """线程安全的结构化 JSON 行日志写入器（每行一个 JSON 对象）。

    服务端日志路径：/home/diff/gwq/logs/tracker_server_YYYYMMDD_HHMMSS.jsonl
    客户端日志路径：/home/diff/gwq/logs/tracker_client_YYYYMMDD_HHMMSS.jsonl
    """

    _instances: dict[str, TrackingLogger] = {}
    _lock = threading.Lock()

    def __init__(self, name: str, log_dir: str = "/gwq/logs"):
        self._name = str(name)
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._path = self._log_dir / f"{self._name}_{ts}.jsonl"
        self._file_lock = threading.Lock()
        self._count = 0
        print(f"[TRACKING_LOG] {self._name} → {self._path}", flush=True)

    @classmethod
    def get(cls, name: str, log_dir: str = "/gwq/logs") -> TrackingLogger:
        with cls._lock:
            if name not in cls._instances:
                cls._instances[name] = cls(name, log_dir=log_dir)
            return cls._instances[name]

    @property
    def path(self) -> Path:
        return self._path

    def log(self, data: dict[str, Any]) -> None:
        """追加一条 JSON 日志记录。"""
        self._count += 1
        record = {
            "_idx": self._count,
            "_ts": time.time(),
            "_isotime": time.strftime("%Y-%m-%dT%H:%M:%S.", time.localtime())
            + f"{time.time() % 1:.6f}"[2:],
        }
        record.update(data)
        line = json.dumps(record, ensure_ascii=False, default=str)
        with self._file_lock:
            with open(self._path, "a", encoding="utf-8") as f:
                f.write(line + "\n")

    def get_count(self) -> int:
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
