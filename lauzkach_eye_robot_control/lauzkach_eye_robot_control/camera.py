"""Frame sources for the overhead camera. Everything here only READS: it sends HTTP GET requests for pictures and never
sends a command to the arm or the controller.

* `MjpegSource`: the RAMION dashboard's live stream (`/cameras/overhead.mjpg?fps=2&profile=preview`), one kept-alive
  connection that reconnects by itself;
* `FileSource`: a picture on disk, for working without the camera.
"""

from __future__ import annotations

import http.client
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

import cv2
import numpy as np

SOI, EOI = b"\xff\xd8", b"\xff\xd9"


class FrameSource:
    """Keeps the newest frame; `latest()` returns (sequence number, JPEG bytes, time received) or None."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame: tuple[int, bytes, float] | None = None
        self._seq = 0
        self.status = "starting"
        self.error = ""

    def _publish(self, jpeg: bytes) -> None:
        with self._lock:
            self._seq += 1
            self._frame = (self._seq, jpeg, time.time())
            self.status, self.error = "live", ""

    def latest(self) -> tuple[int, bytes, float] | None:
        with self._lock:
            return self._frame

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None:
            self._thread.join(timeout=3)


class MjpegSource(FrameSource):
    def __init__(self, url: str, timeout: float = 8.0) -> None:
        super().__init__()
        parts = urlsplit(url)
        if parts.scheme != "http" or not parts.hostname:
            raise ValueError("the camera URL must be http://host:port/path")
        self.host, self.port = parts.hostname, parts.port or 80
        self.path = parts.path + (f"?{parts.query}" if parts.query else "")
        self.timeout = timeout

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="camera")
        self._thread.start()

    def _run(self) -> None:
        delay = 1.0
        while not self._stopping.is_set():
            conn = None
            try:
                self.status = "connecting"
                conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
                conn.request("GET", self.path)  # a plain GET of the camera stream: read-only
                response = conn.getresponse()
                if response.status != 200:
                    raise RuntimeError(f"camera answered HTTP {response.status}")
                buffer = b""
                while not self._stopping.is_set():
                    chunk = response.read1(65536) if hasattr(response, "read1") else response.read(4096)
                    if not chunk:
                        raise ConnectionError("camera stream ended")
                    buffer += chunk
                    while True:
                        start = buffer.find(SOI)
                        end = buffer.find(EOI, start + 2) if start >= 0 else -1
                        if start < 0 or end < 0:
                            break
                        self._publish(buffer[start:end + 2])
                        buffer = buffer[end + 2:]
                        delay = 1.0
                    if len(buffer) > 4_000_000:  # never grow without bound if the stream is garbage
                        buffer = b""
            except (OSError, RuntimeError, http.client.HTTPException) as exc:
                self.status, self.error = "reconnecting", f"{type(exc).__name__}: {exc}"
            finally:
                if conn is not None:
                    conn.close()
            self._stopping.wait(delay)
            delay = min(delay * 2, 10.0)


class FileSource(FrameSource):
    """One picture from disk, published again every second so the pipeline behaves as with a live camera."""

    def __init__(self, path: str, interval: float = 1.0) -> None:
        super().__init__()
        self.interval = interval
        self.path = Path(path)
        self.jpeg = self.path.read_bytes() if self.path.suffix.lower() in (".jpg", ".jpeg") else cv2.imencode(
            ".jpg", cv2.imread(str(self.path)))[1].tobytes()

    def start(self) -> None:
        def run() -> None:
            while not self._stopping.is_set():
                self._publish(self.jpeg)
                self._stopping.wait(self.interval)
        self._thread = threading.Thread(target=run, daemon=True, name="file-camera")
        self._thread.start()


def decode(jpeg: bytes) -> np.ndarray | None:
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
