"""Write structured monotonic timing records for robot experiments."""

from contextlib import contextmanager
from contextvars import ContextVar
import json
import os
from pathlib import Path
import sys
import threading
import time


_BOOK_INDEX = ContextVar("fpc_experiment_book_index", default=None)
_WRITE_LOCK = threading.Lock()


def _emit(record):
    payload = {
        "run_id": os.environ.get("FPC_EXPERIMENT_RUN_ID"),
        "commit": os.environ.get("FPC_DEPLOYED_COMMIT"),
        "book_index": _BOOK_INDEX.get(),
        **record,
    }
    line = json.dumps(payload, ensure_ascii=False, sort_keys=True, default=str)
    print(f"[计时] {line}", flush=True)

    record_dir = os.environ.get("FPC_EXPERIMENT_RECORD_DIR")
    if not record_dir:
        return
    path = Path(record_dir) / "timings.jsonl"
    try:
        with _WRITE_LOCK:
            with path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")
    except OSError as error:
        print(f"[计时] 写入 {path} 失败: {error}", file=sys.stderr, flush=True)


@contextmanager
def book_scope(book_index):
    token = _BOOK_INDEX.set(int(book_index))
    try:
        yield
    finally:
        _BOOK_INDEX.reset(token)


@contextmanager
def timed_phase(phase, **fields):
    started_ns = time.monotonic_ns()
    details = dict(fields)
    ok = False
    error_type = None
    try:
        yield details
        ok = True
    except BaseException as error:
        error_type = type(error).__name__
        raise
    finally:
        finished_ns = time.monotonic_ns()
        record = {
            "phase": str(phase),
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": finished_ns,
            "duration_s": round((finished_ns - started_ns) / 1_000_000_000.0, 6),
            "ok": ok,
            **details,
        }
        if error_type is not None:
            record["error_type"] = error_type
        _emit(record)


def timed_call(phase, operation, **fields):
    with timed_phase(phase, **fields):
        return operation()
