"""Disconnected preview has no backend and never fabricates telemetry."""

from __future__ import annotations

import json
import threading
from http.client import HTTPConnection

import pytest

import soarm
from soarm.panel import PanelError, PanelServer, RobotDescription
from soarm.panel_preview import PreviewController
from soarm.panel_viewer import measured_pose


@pytest.fixture
def preview(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Disconnected preview must not construct a hardware backend")

    monkeypatch.setattr("soarm.panel.FeetechArm", forbidden)
    monkeypatch.setattr("soarm.feetech.FeetechBus", forbidden)
    controller = PreviewController(RobotDescription.load(soarm.ROBOT_DIR))
    controller.start()
    yield controller
    controller.close()


def test_preview_has_no_pose_or_control_loop(preview):
    assert not hasattr(preview, "arm")
    assert not hasattr(preview, "_thread")
    state = preview.state()
    assert state["preview_only"]
    assert state["measured"] == state["command"] == state["target"] == {}
    assert state["port"] is None
    assert state["age_s"] is None
    assert state["sample"] == {
        "age_s": None, "sequence": 0, "loop_ok": False, "connected": False,
    }
    assert state["torque"] is None  # Preview has not read physical torque state.
    assert state["loop_hz"] == 0
    with pytest.raises(ValueError, match="No finite measurement"):
        measured_pose(state, preview.description.joints)
    preview.stop()
    preview.close()
    preview.close()
    assert preview.state() == state


def test_direct_control_is_rejected(preview):
    operations = [
        lambda: preview.claim("Observer"),
        lambda: preview.heartbeat("token"),
        lambda: preview.release("token"),
        lambda: preview._authorize("token"),
        lambda: preview.jog("shoulder_pan", 1),
        lambda: preview.set_target("shoulder_pan", 0),
        preview.home,
        lambda: preview.set_torque(True),
        lambda: preview.set_torque(False),
        lambda: preview.set_speed(1),
    ]
    for operation in operations:
        with pytest.raises(PanelError, match="disconnected preview"):
            operation()


def test_http_observers_and_stop_work_without_hardware(preview):
    server = PanelServer(("127.0.0.1", 0), preview)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def request(method, path, body=None):
        connection = HTTPConnection("127.0.0.1", server.server_port, timeout=5)
        try:
            connection.request(method, path, json.dumps(body) if body is not None else None,
                               {"Content-Type": "application/json"})
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    try:
        for path in ("/api/state", "/api/telemetry"):
            status, body = request("GET", path)
            state = json.loads(body)
            assert status == 200
            assert not state["connected"] and not state["loop_ok"] and not state["torque"]
            assert state["measured"] == {}
            assert state["control"]["owner_id"] is None
            assert state["cameras"]["overhead"]["status"] == "unconfigured"
        assert request("GET", "/")[0] == 307
        assert request("POST", "/api/stop", {})[0] == 200
        commands = {
            "control/claim": {"label": "Observer"},
            "control/heartbeat": {"token": "token"},
            "control/release": {"token": "token"},
            "jog": {"token": "token", "joint": "shoulder_pan", "delta": 1},
            "target": {"token": "token", "joint": "shoulder_pan", "value": 0},
            "home": {"token": "token"},
            "torque": {"token": "token", "on": True},
            "speed": {"token": "token", "value": 1},
            "cameras": {"token": "token", "overhead": 0, "wrist": 2},
        }
        for command, body in commands.items():
            assert request("POST", f"/api/{command}", body)[0] == 409
        assert preview.state()["measured"] == {}
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_preview_main_skips_serial_detection_and_hardware(monkeypatch):
    from soarm import panel

    lifecycle = []

    def forbidden(*args, **kwargs):
        pytest.fail("--preview must not detect, construct, open or write arm hardware")

    monkeypatch.setattr(panel, "detect_port", forbidden)
    monkeypatch.setattr(panel, "FeetechArm", forbidden)
    monkeypatch.setattr(panel, "PanelController", forbidden)
    monkeypatch.setattr("serial.tools.list_ports.comports", forbidden)
    for method in ("open", "write", "sync_write"):
        monkeypatch.setattr(f"soarm.feetech.FeetechBus.{method}", forbidden)

    class Cameras:
        def __init__(self, overhead, wrist, observer=None):
            assert (overhead, wrist, observer) == (0, 2, None)
            lifecycle.append("cameras created")

        def start(self):
            lifecycle.append("cameras started")

        def close(self):
            lifecycle.append("cameras closed")

    class Server:
        server_port = 8765

        def __init__(self, address, controller, cameras, viewer, *, dashboard_url):
            assert address == ("127.0.0.1", 8765)
            assert dashboard_url == "http://127.0.0.1:8780/"
            assert isinstance(controller, PreviewController)
            assert controller.state()["measured"] == {}
            assert not controller.state()["connected"]
            assert viewer is None
            self.cameras = cameras
            lifecycle.append("server created")

        def serve_forever(self):
            lifecycle.append("server serving")
            raise KeyboardInterrupt

        def server_close(self):
            lifecycle.append("server closed")
            self.cameras.close()

    monkeypatch.setattr(panel, "CameraManager", Cameras)
    monkeypatch.setattr(panel, "PanelServer", Server)
    assert panel.main([
        "--preview", "--no-viewer", "--no-browser",
        "--overhead-camera", "0", "--wrist-camera", "2",
    ]) == 0
    assert lifecycle == [
        "cameras created", "server created", "cameras started", "server serving",
        "server closed", "cameras closed",
    ]


def test_persisted_emergency_stop_prevents_arm_startup_and_preview_can_reset(tmp_path, monkeypatch):
    from soarm import panel

    state_file = tmp_path / "state" / "emergency-stop.json"
    monkeypatch.setenv("RAMION_STOP_STATE_FILE", str(state_file))
    description = RobotDescription.load(soarm.ROBOT_DIR)
    preview = PreviewController(description, stop_state=panel.EmergencyStopState.load(state_file))
    latched = panel.run_command(preview, "emergency-stop", {})
    assert latched["emergency_stop"]["latched"]
    assert latched["emergency_stop"]["generation"] == 1
    assert latched["emergency_stop"]["hold_status"] == "unavailable"
    repeated = panel.run_command(preview, "emergency-stop", {})
    assert repeated["emergency_stop"] == {**latched["emergency_stop"], "generation": 2}

    def forbidden(*args, **kwargs):
        pytest.fail("A persisted emergency stop must block startup before opening the arm")

    monkeypatch.setattr(panel, "FeetechArm", forbidden)
    assert panel.main(["--port", "COM_TEST", "--no-viewer", "--no-browser"]) == 2

    restarted = PreviewController(description, stop_state=panel.EmergencyStopState.load(state_file))
    assert restarted.state()["emergency_stop"]["latched"]
    reset = {"emergency_stop_generation": 2}
    cleared = panel.run_command(restarted, "emergency-stop/reset", reset)
    assert not cleared["emergency_stop"]["latched"]
    assert panel.run_command(restarted, "emergency-stop/reset", reset) == cleared
    assert panel.EmergencyStopState.load(state_file).snapshot() == cleared["emergency_stop"]


def test_corrupt_emergency_stop_state_starts_latched(tmp_path):
    from soarm.panel import EmergencyStopState

    state_file = tmp_path / "emergency-stop.json"
    state_file.write_text("{invalid JSON", encoding="utf-8")
    assert EmergencyStopState.load(state_file).latched
