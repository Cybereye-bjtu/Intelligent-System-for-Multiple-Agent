import threading
import time

import pytest

from aaa_navigation.persistent_detection_protocol import (
    DirectoryRequestClient,
    serve_requests,
)


def test_persistent_handler_is_reused_for_multiple_requests(tmp_path):
    stop = threading.Event()

    class Handler:
        def __init__(self):
            self.calls = 0

        def __call__(self, request):
            self.calls += 1
            return {
                "success": True,
                "prompt": request["prompt"],
                "call": self.calls,
            }

    handler = Handler()
    thread = threading.Thread(
        target=serve_requests,
        args=(tmp_path, handler),
        kwargs={"stop_requested": stop.is_set, "poll_interval": 0.005},
        daemon=True,
    )
    thread.start()
    client = DirectoryRequestClient(tmp_path)
    client.wait_until_ready(1.0)
    first = client.request({"prompt": "white box"}, 1.0)
    second = client.request({"prompt": "black vehicle"}, 1.0)
    stop.set()
    thread.join(timeout=1.0)

    assert first["call"] == 1
    assert second["call"] == 2
    assert handler.calls == 2


def test_handler_failure_does_not_stop_the_worker(tmp_path):
    stop = threading.Event()

    def handler(request):
        if request["prompt"] == "bad":
            raise RuntimeError("synthetic failure")
        return {"success": True, "prompt": request["prompt"]}

    thread = threading.Thread(
        target=serve_requests,
        args=(tmp_path, handler),
        kwargs={"stop_requested": stop.is_set, "poll_interval": 0.005},
        daemon=True,
    )
    thread.start()
    client = DirectoryRequestClient(tmp_path)
    client.wait_until_ready(1.0)
    failed = client.request({"prompt": "bad"}, 1.0)
    recovered = client.request({"prompt": "good"}, 1.0)
    stop.set()
    thread.join(timeout=1.0)

    assert failed["success"] is False
    assert "synthetic failure" in failed["reason"]
    assert recovered == {
        "success": True,
        "prompt": "good",
        "request_id": recovered["request_id"],
    }


def test_wait_until_ready_times_out(tmp_path):
    client = DirectoryRequestClient(tmp_path)
    with pytest.raises(TimeoutError):
        client.wait_until_ready(0.05)
