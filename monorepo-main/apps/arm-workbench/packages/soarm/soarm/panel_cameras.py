"""Local USB camera capture, isolated from the robot control bus.

Importing this module never imports OpenCV or opens a device. Capture workers
cache JPEG bytes; HTTP clients must only read the cache, never call read().
"""

from __future__ import annotations

import importlib
import os
import re
import sys
import threading
import time
from collections.abc import Callable
from typing import Any

NAMES = ("overhead", "wrist", "observer")
CameraSource = int | str
STALE_AFTER_S = 2.0
SHUTDOWN_TIMEOUT_S = 1.0
# DirectShow graph negotiation is unreliable when multiple devices open at once.
_CAPTURE_OPEN_LOCK = threading.Lock()


def _device_identity(source: CameraSource) -> str:
    path = f"/dev/video{source}" if type(source) is int else source
    # Resolve Linux udev aliases without requiring the camera to be plugged in.
    resolved = os.path.realpath(path) if sys.platform.startswith("linux") else path
    if not re.fullmatch(r"/dev/video[0-9]+", resolved) and resolved != path:
        raise ValueError("Camera device aliases must resolve to /dev/videoN")
    return resolved


def parse_camera_source(value: str) -> CameraSource:
    """Parse a CLI index or an absolute Linux V4L2 device path."""
    source = int(value) if value.isascii() and value.isdecimal() else value
    _validate(source, None)
    return source


def _validate(
    overhead: CameraSource | None,
    wrist: CameraSource | None,
    observer: CameraSource | None = None,
) -> dict[str, CameraSource | None]:
    sources = dict(zip(NAMES, (overhead, wrist, observer), strict=True))
    assigned: dict[str, str] = {}
    for name, source in sources.items():
        if source is None:
            continue
        if not (
            (type(source) is int and source >= 0)
            or (type(source) is str and re.fullmatch(
                r"/dev/(?:video[0-9]+|v4l/by-id/[^/.][^/\x00]*)", source
            ) and "\n" not in source and "\r" not in source)
        ):
            raise ValueError(
                "Camera sources must be nonnegative indices, /dev/videoN or /dev/v4l/by-id paths, or null"
            )
        identity = _device_identity(source)
        if identity in assigned:
            raise ValueError(
                f"{assigned[identity].capitalize()} and {name} cameras must use different devices"
            )
        assigned[identity] = name
    return sources


class _Worker:
    def __init__(self, source: CameraSource, factory: Callable, encode: Callable, clock: Callable):
        self.source, self.factory, self.encode, self.clock = source, factory, encode, clock
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.state = "connecting"
        self.error: str | None = None
        self.last_frame: float | None = None
        self.width = self.height = None
        self.fps: float | None = None
        self.sequence = 0
        self.data: bytes | None = None
        self.thread = threading.Thread(target=self.run, name=f"camera-{source}", daemon=True)

    def run(self) -> None:
        delay = 0.25
        while not self.stop.is_set():
            capture = None
            try:
                capture = self.factory(self.source)
                if not capture.isOpened():
                    raise RuntimeError(
                        f"Cannot open local camera {self.source}; check its index and other applications"
                    )
                while not self.stop.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise RuntimeError(f"Camera {self.source} stopped returning frames")
                    if self.stop.is_set():
                        break
                    data = self.encode(frame)
                    now = self.clock()
                    with self.lock:
                        if self.stop.is_set():
                            break
                        previous = self.last_frame
                        self.data = data
                        self.sequence += 1
                        self.last_frame = now
                        self.height, self.width = map(int, frame.shape[:2])
                        self.fps = (
                            (1.0 / (now - previous)) if previous is not None and now > previous else None
                        )
                        self.state, self.error = "live", None
                    delay = 0.25
                    # Prevent a defective/virtual capture from busy-spinning.
                    if self.stop.wait(0.001):
                        break
            except Exception as exc:
                with self.lock:
                    self.state, self.error = "unavailable", str(exc)
            finally:
                if capture is not None:
                    try:
                        capture.release()
                    except Exception:
                        pass
            if not self.stop.is_set():
                self.stop.wait(delay)
                delay = min(delay * 2, 4.0)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            age = None if self.last_frame is None else max(0.0, self.clock() - self.last_frame)
            state = self.state
            if state == "live" and age is not None and age > STALE_AFTER_S:
                state = "stale"
            return dict(
                source=self.source,
                status=state,
                age_s=age,
                width=self.width,
                height=self.height,
                fps=self.fps,
                error=self.error,
            )

    def jpeg(self) -> tuple[int, bytes] | None:
        with self.lock:
            if (
                self.state != "live"
                or self.data is None
                or self.last_frame is None
                or self.clock() - self.last_frame > STALE_AFTER_S
            ):
                return None
            return self.sequence, self.data


class CameraManager:
    """Named local cameras with unique devices and bounded worker shutdown.

    A native driver read/release may hang. Reconfiguration refuses to launch a
    replacement while an old worker is still alive; close never waits forever.
    The optional factory/encoder/clock hooks are for hardware-free unit tests.
    """

    def __init__(
        self,
        overhead: CameraSource | None = None,
        wrist: CameraSource | None = None,
        observer: CameraSource | None = None,
        *,
        capture_factory: Callable | None = None,
        encoder: Callable | None = None,
        clock: Callable = time.monotonic,
    ):
        self._sources = _validate(overhead, wrist, observer)
        self._factory, self._encoder, self._clock = capture_factory, encoder, clock
        self._lock = threading.RLock()
        self._operations = threading.Lock()
        self._workers: dict[str, _Worker] = {}
        self._generations = {name: 0 for name in NAMES}
        self._started = False
        self._closed = False

    @staticmethod
    def _opencv():
        try:
            return importlib.import_module("cv2")
        except ImportError as exc:
            raise RuntimeError(
                f"OpenCV import failed: {exc}. "
                "Check opencv-python/opencv-contrib-python and Linux shared libraries."
            ) from exc

    def _capture(self, source: CameraSource):
        if self._factory is not None:
            return self._factory(source)
        with _CAPTURE_OPEN_LOCK:
            cv2 = self._opencv()
            if sys.platform == "win32":
                capture = cv2.VideoCapture(source, cv2.CAP_DSHOW)
            elif sys.platform.startswith("linux"):
                capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
            else:
                capture = cv2.VideoCapture(source)
            try:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                capture.set(cv2.CAP_PROP_FPS, 15)
                # Resolution negotiation can silently reset MJPG to raw YUY2.
                # Apply compression last to reduce USB bandwidth across cameras.
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            except Exception:
                capture.release()
                raise
            return capture

    def _encode(self, frame) -> bytes:
        if self._encoder is not None:
            return self._encoder(frame)
        ok, encoded = self._opencv().imencode(".jpg", frame)
        if not ok:
            raise RuntimeError("Camera JPEG encoding failed")
        return encoded.tobytes()

    def _launch(self, name: str, source: CameraSource) -> None:
        worker = _Worker(source, self._capture, self._encode, self._clock)
        self._workers[name] = worker
        self._generations[name] += 1
        worker.thread.start()

    def start(self) -> None:
        with self._operations, self._lock:
            if self._closed:
                raise RuntimeError("Camera manager is closed")
            if self._started:
                return
            self._started = True
            for name, source in self._sources.items():
                if source is not None:
                    self._launch(name, source)

    def configure(
        self,
        overhead: CameraSource | None,
        wrist: CameraSource | None,
        observer: CameraSource | None = None,
    ) -> None:
        sources = _validate(overhead, wrist, observer)
        with self._operations:
            with self._lock:
                if self._closed:
                    raise RuntimeError("Camera manager is closed")
                changed = [
                    name
                    for name in NAMES
                    if sources[name] != self._sources[name]
                    or (name in self._workers and self._workers[name].stop.is_set())
                ]
                old = [self._workers[name] for name in changed if name in self._workers]
                for worker in old:
                    worker.stop.set()
            # Do not hold the status/cache lock while waiting for device drivers.
            deadline = time.monotonic() + SHUTDOWN_TIMEOUT_S
            for worker in old:
                worker.thread.join(max(0.0, deadline - time.monotonic()))
            if any(worker.thread.is_alive() for worker in old):
                for worker in old:
                    with worker.lock:
                        worker.state = "unavailable"
                        worker.error = "Camera driver did not stop; retry configuration after it exits"
                raise RuntimeError("Camera driver did not stop; replacement was not opened")
            with self._lock:
                self._sources = sources
                for name in changed:
                    self._workers.pop(name, None)
                    if self._started and sources[name] is not None:
                        self._launch(name, sources[name])

    def status(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            result = {}
            for name in NAMES:
                worker = self._workers.get(name)
                result[name] = (
                    worker.snapshot()
                    if worker
                    else dict(
                        source=self._sources[name],
                        status="connecting" if self._sources[name] is not None else "unconfigured",
                        age_s=None,
                        width=None,
                        height=None,
                        fps=None,
                        error=None,
                    )
                )
                result[name]["url"] = f"/cameras/{name}.mjpg"
            return result

    def jpeg(self, name: str) -> tuple[int, bytes] | None:
        if name not in NAMES:
            raise ValueError("Unknown camera name")
        with self._lock:
            worker = self._workers.get(name)
            frame = worker.jpeg() if worker else None
            return (self._generations[name] * 10**12 + frame[0], frame[1]) if frame else None

    def close(self) -> None:
        with self._operations:
            with self._lock:
                self._closed = True
                workers = list(self._workers.values())
                for worker in workers:
                    worker.stop.set()
                    with worker.lock:
                        worker.state, worker.error = "unavailable", "Camera manager is closed"
            deadline = time.monotonic() + SHUTDOWN_TIMEOUT_S
            for worker in workers:
                worker.thread.join(max(0.0, deadline - time.monotonic()))
