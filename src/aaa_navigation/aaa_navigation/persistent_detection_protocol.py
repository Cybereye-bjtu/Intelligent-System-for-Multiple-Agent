"""Small file-backed RPC protocol for a persistent inference subprocess."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid


REQUEST_SUFFIX = ".request.json"
RESULT_SUFFIX = ".result.json"
READY_FILE = "ready.json"
STARTUP_ERROR_FILE = "startup_error.json"


def atomic_write_json(path: Path, value: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False) + "\n")
    os.replace(temporary, path)


def serve_requests(request_dir, handler, *, stop_requested=None, poll_interval=0.05):
    """Serve requests after the expensive model has already been constructed."""
    directory = Path(request_dir)
    directory.mkdir(parents=True, exist_ok=True)
    atomic_write_json(
        directory / READY_FILE,
        {"ready": True, "pid": os.getpid(), "loaded_at": time.time()},
    )
    should_stop = stop_requested or (lambda: False)
    while not should_stop():
        requests = sorted(directory.glob(f"*{REQUEST_SUFFIX}"))
        if not requests:
            time.sleep(float(poll_interval))
            continue
        for request_path in requests:
            request_id = request_path.name.removesuffix(REQUEST_SUFFIX)
            try:
                request = json.loads(request_path.read_text())
                response = handler(request)
                if not isinstance(response, dict):
                    raise TypeError("persistent detector handler must return a dictionary")
            except Exception as error:
                response = {"success": False, "reason": str(error)}
            finally:
                request_path.unlink(missing_ok=True)
            response.setdefault("request_id", request_id)
            atomic_write_json(directory / f"{request_id}{RESULT_SUFFIX}", response)


class DirectoryRequestClient:
    def __init__(self, request_dir):
        self.directory = Path(request_dir)
        self.directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def wait_until_ready(self, timeout, *, process=None):
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            error_path = self.directory / STARTUP_ERROR_FILE
            if error_path.is_file():
                value = json.loads(error_path.read_text())
                raise RuntimeError(value.get("reason", "detector startup failed"))
            if (self.directory / READY_FILE).is_file():
                return
            if process is not None and process.poll() is not None:
                raise RuntimeError(
                    f"persistent detector exited during startup with code {process.returncode}"
                )
            time.sleep(0.05)
        raise TimeoutError(f"persistent detector was not ready within {float(timeout):.1f}s")

    def request(self, value, timeout, *, process=None):
        with self._lock:
            request_id = uuid.uuid4().hex
            request_path = self.directory / f"{request_id}{REQUEST_SUFFIX}"
            result_path = self.directory / f"{request_id}{RESULT_SUFFIX}"
            atomic_write_json(request_path, dict(value))
            deadline = time.monotonic() + float(timeout)
            while time.monotonic() < deadline:
                if result_path.is_file():
                    result = json.loads(result_path.read_text())
                    result_path.unlink(missing_ok=True)
                    return result
                if process is not None and process.poll() is not None:
                    request_path.unlink(missing_ok=True)
                    raise RuntimeError(
                        f"persistent detector exited with code {process.returncode}"
                    )
                time.sleep(0.05)
            request_path.unlink(missing_ok=True)
            raise TimeoutError(f"persistent detection exceeded {float(timeout):.1f}s")


class PersistentWorkerProcess:
    """Own a model process whose requests do not recreate the model."""

    def __init__(self, command, *, environment=None, startup_timeout=240.0):
        self._temporary = tempfile.TemporaryDirectory(prefix="aaa_detector_rpc_")
        self.request_dir = Path(self._temporary.name)
        worker_environment = os.environ.copy()
        worker_environment.update(
            {str(key): str(value) for key, value in (environment or {}).items()}
        )
        self.process = subprocess.Popen(
            [*command, "--serve-dir", str(self.request_dir)],
            env=worker_environment,
            start_new_session=True,
        )
        self.client = DirectoryRequestClient(self.request_dir)
        self.startup_timeout = float(startup_timeout)
        self._ready = False
        self._closed = False

    def request(self, value, *, timeout):
        if self._closed:
            raise RuntimeError("persistent detector is closed")
        if not self._ready:
            self.client.wait_until_ready(
                self.startup_timeout,
                process=self.process,
            )
            self._ready = True
        return self.client.request(value, timeout, process=self.process)

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if self.process.poll() is None:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5.0)
        self._temporary.cleanup()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
