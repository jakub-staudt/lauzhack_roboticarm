"""Camera manager tests use fake captures only; no hardware or OpenCV needed."""

import threading
import time
from types import SimpleNamespace

import pytest

from soarm.panel_cameras import STALE_AFTER_S, CameraManager, parse_camera_source


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    assert predicate()


class Capture:
    def __init__(self, *, fail=False, gate=None):
        self.fail, self.gate = fail, gate
        self.released = False
        self.reads = 0

    def isOpened(self):
        return True

    def read(self):
        self.reads += 1
        if self.gate is not None:
            self.gate.wait()
        return (False, None) if self.fail else (True, SimpleNamespace(shape=(480, 640, 3)))

    def release(self):
        self.released = True


def test_lazy_start_and_cached_jpeg():
    opened, encoded = [], []
    capture = Capture()

    def factory(source):
        opened.append(source)
        return capture

    def encode(frame):
        encoded.append(frame)
        return b"jpeg"

    manager = CameraManager(0, capture_factory=factory, encoder=encode)
    assert opened == []
    assert manager.status()["wrist"]["status"] == "unconfigured"
    try:
        manager.start()
        manager.start()
        eventually(lambda: manager.jpeg("overhead") is not None)
        assert opened == [0]
        status = manager.status()["overhead"]
        assert status["status"] == "live"
        assert (status["width"], status["height"]) == (640, 480)
        assert status["url"] == "/cameras/overhead.mjpg"
        assert manager.jpeg("overhead")[1] == b"jpeg"
        # Clients read the existing bytes without JPEG re-encoding.
        before = len(encoded)
        for _ in range(30):
            manager.jpeg("overhead")
        assert len(encoded) == before
    finally:
        manager.close()
    assert capture.released
    assert manager.jpeg("overhead") is None


@pytest.mark.parametrize(
    "sources", [(0, 0), (-1, None), (True, None), ("http://host", None), ("video.mp4", None), (1.2, None)]
)
def test_invalid_assignment_is_atomic(sources):
    manager = CameraManager(1, 2)
    with pytest.raises(ValueError):
        manager.configure(*sources)
    assert manager.status()["overhead"]["source"] == 1
    assert manager.status()["wrist"]["source"] == 2


@pytest.mark.parametrize("source", [
    "/tmp/video.mp4", "/dev/null", "file:///dev/video0", "0", "/dev/video-1",
    "/dev/v4l/by-id/../video0", "/dev/v4l/by-id/camera/extra", "/dev/video0\n",
])
def test_non_camera_paths_rejected(source):
    with pytest.raises(ValueError):
        CameraManager(source)


@pytest.mark.parametrize("value, expected", [
    ("0", 0), ("12", 12), ("/dev/video2", "/dev/video2"),
    ("/dev/v4l/by-id/usb-camera-video-index0", "/dev/v4l/by-id/usb-camera-video-index0"),
])
def test_cli_source_parser(value, expected):
    assert parse_camera_source(value) == expected


def test_linux_aliases_cannot_open_same_device_twice(monkeypatch):
    from soarm import panel_cameras

    monkeypatch.setattr(panel_cameras.sys, "platform", "linux")
    aliases = {
        "/dev/v4l/by-id/camera-one": "/dev/video0",
        "/dev/v4l/by-id/camera-two": "/dev/video0",
    }
    monkeypatch.setattr(panel_cameras.os.path, "realpath", lambda path: aliases.get(path, path))
    manager = CameraManager(1, 2, 3)
    for sources in [
        (0, "/dev/video0", None),
        (0, 1, "/dev/v4l/by-id/camera-one"),
        (1, "/dev/v4l/by-id/camera-one", "/dev/v4l/by-id/camera-two"),
    ]:
        with pytest.raises(ValueError, match="different devices"):
            manager.configure(*sources)
        assert [manager.status()[name]["source"] for name in ("overhead", "wrist", "observer")] == [1, 2, 3]


def test_alias_to_non_video_file_rejected(monkeypatch):
    from soarm import panel_cameras

    monkeypatch.setattr(panel_cameras.sys, "platform", "linux")
    monkeypatch.setattr(panel_cameras.os.path, "realpath", lambda _: "/tmp/movie.mp4")
    with pytest.raises(ValueError, match="resolve to /dev/videoN"):
        CameraManager("/dev/v4l/by-id/camera")


def test_stable_camera_path_is_preserved_for_capture_and_status(monkeypatch):
    from soarm import panel_cameras

    monkeypatch.setattr(panel_cameras.sys, "platform", "linux")
    monkeypatch.setattr(panel_cameras.os.path, "realpath", lambda path: path)
    source = "/dev/v4l/by-id/usb-camera-video-index0"
    opened = []
    capture = Capture()
    manager = CameraManager(source, capture_factory=lambda path: (opened.append(path), capture)[1],
                            encoder=lambda _: b"jpeg")
    try:
        manager.start()
        eventually(lambda: manager.jpeg("overhead") is not None)
        assert opened == [source]
        assert manager.status()["overhead"]["source"] == source
    finally:
        manager.close()


def test_staleness_uses_monotonic_age():
    now = [100.0]
    gate = threading.Event()

    class OneFrame(Capture):
        def read(self):
            if self.reads:
                gate.wait()
            return super().read()

    capture = OneFrame()
    manager = CameraManager(
        0, capture_factory=lambda _: capture, encoder=lambda _: b"jpeg", clock=lambda: now[0]
    )
    try:
        manager.start()
        eventually(lambda: capture.reads > 0 and manager.jpeg("overhead") is not None)
        now[0] += STALE_AFTER_S + 1
        assert manager.status()["overhead"]["status"] == "stale"
        assert manager.jpeg("overhead") is None
    finally:
        gate.set()
        manager.close()


def test_failed_capture_recovers_and_releases():
    captures = []

    def factory(_):
        capture = Capture(fail=not captures)
        captures.append(capture)
        return capture

    manager = CameraManager(0, capture_factory=factory, encoder=lambda _: b"jpeg")
    try:
        manager.start()
        eventually(lambda: manager.status()["overhead"]["status"] == "unavailable")
        assert manager.jpeg("overhead") is None
        eventually(lambda: manager.status()["overhead"]["status"] == "live")
        assert captures[0].released
        assert manager.status()["overhead"]["error"] is None
    finally:
        manager.close()


def test_swap_stops_both_old_devices_before_opening_replacements():
    captures, opened = [], []

    def factory(source):
        if len(opened) >= 2:
            assert all(c.released for c in captures[:2])
        opened.append(source)
        capture = Capture()
        captures.append(capture)
        return capture

    manager = CameraManager(0, 1, capture_factory=factory, encoder=lambda _: b"jpeg")
    try:
        manager.start()
        eventually(lambda: len(opened) == 2)
        manager.configure(1, 0)
        eventually(lambda: len(opened) == 4)
        assert manager.status()["overhead"]["source"] == 1
        assert manager.status()["wrist"]["source"] == 0
    finally:
        manager.close()


def test_missing_opencv_is_actionable(monkeypatch):
    def missing(_):
        raise ImportError("absent")

    monkeypatch.setattr("soarm.panel_cameras.importlib.import_module", missing)
    manager = CameraManager(0)
    try:
        manager.start()
        eventually(lambda: manager.status()["overhead"]["status"] == "unavailable")
        assert "opencv-python" in manager.status()["overhead"]["error"]
    finally:
        manager.close()


def test_hung_driver_shutdown_is_bounded_and_blocks_replacement(monkeypatch):
    monkeypatch.setattr("soarm.panel_cameras.SHUTDOWN_TIMEOUT_S", 0.03)
    gate = threading.Event()
    captures = []

    def factory(_):
        capture = Capture(gate=gate)
        captures.append(capture)
        return capture

    manager = CameraManager(0, capture_factory=factory, encoder=lambda _: b"jpeg")
    try:
        manager.start()
        eventually(lambda: captures and captures[0].reads)
        with pytest.raises(RuntimeError, match="replacement was not opened"):
            manager.configure(1, None)
        assert len(captures) == 1
        assert manager.jpeg("overhead") is None
        before = time.monotonic()
        manager.close()
        assert time.monotonic() - before < 0.5
    finally:
        gate.set()
        manager.close()


def test_unknown_camera_rejected():
    with pytest.raises(ValueError):
        CameraManager().jpeg("other")


@pytest.mark.parametrize("platform, backend, source", [
    ("win32", 700, 2), ("linux", 200, 2), ("linux", 200, "/dev/video2"),
])
def test_backend_and_initial_capture_settings(monkeypatch, platform, backend, source):
    from soarm import panel_cameras

    calls = []
    capture = Capture()
    capture.set = lambda key, value: calls.append((key, value))
    cv2 = SimpleNamespace(
        CAP_PROP_FOURCC=6,
        VideoWriter_fourcc=lambda *args: 1196444237,
        CAP_DSHOW=700,
        CAP_V4L2=200,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        VideoCapture=lambda *args: (calls.append(args), capture)[1],
    )
    monkeypatch.setattr(panel_cameras.sys, "platform", platform)
    monkeypatch.setattr(panel_cameras.os.path, "realpath", lambda path: path)
    monkeypatch.setattr(panel_cameras.importlib, "import_module", lambda _: cv2)
    manager = CameraManager(source, encoder=lambda _: b"jpeg")
    try:
        manager.start()
        eventually(lambda: manager.jpeg("overhead") is not None)
        assert calls == [(source, backend), (3, 640), (4, 480), (5, 15), (6, 1196444237)]
    finally:
        manager.close()


def test_read_failure_hides_previously_cached_frame():
    capture = Capture()
    manager = CameraManager(0, capture_factory=lambda _: capture, encoder=lambda _: b"jpeg")
    try:
        manager.start()
        eventually(lambda: manager.jpeg("overhead") is not None)
        capture.fail = True
        eventually(lambda: manager.status()["overhead"]["status"] == "unavailable")
        assert manager.jpeg("overhead") is None
        assert manager.status()["overhead"]["age_s"] is not None
    finally:
        manager.close()


def test_optional_observer_can_fail_and_be_reconfigured_without_reopening_core_cameras():
    captures = {0: Capture(), 1: Capture(), 2: Capture(fail=True), 3: Capture()}
    manager = CameraManager(
        0, 1, capture_factory=captures.__getitem__, encoder=lambda _: b"jpeg"
    )
    try:
        assert manager.status()["observer"]["status"] == "unconfigured"
        assert manager.jpeg("observer") is None
        manager.start()
        manager.configure(0, 1, 2)
        eventually(lambda: manager.status()["observer"]["status"] == "unavailable")
        for name in ("overhead", "wrist"):
            eventually(lambda name=name: manager.jpeg(name) is not None)
        assert manager.jpeg("observer") is None
        manager.configure(0, 1, 3)
        eventually(lambda: manager.jpeg("observer") is not None)
        assert manager.jpeg("observer")[1] == b"jpeg"
        assert manager.status()["observer"]["url"] == "/cameras/observer.mjpg"
        manager.configure(0, 1, None)
        assert manager.status()["observer"]["status"] == "unconfigured"
        assert manager.jpeg("observer") is None
        assert captures[3].released
        assert not captures[0].released and not captures[1].released
    finally:
        manager.close()
