"""The test panel (`soarm-panel`) on the fake bus, which keeps each servo where the test puts it."""

from __future__ import annotations

import json
import threading
from collections.abc import Iterator
from http.client import HTTPConnection, HTTPResponse

import pytest
from machinekind_twin_api.backend import BackendContext

import soarm
from fake_bus import REST_TICKS, FakeBusLink
from soarm import feetech
from soarm.panel import (
    DEFAULT_SPEED,
    HEARTBEAT_TIMEOUT_S,
    MAX_DT_S,
    MODEL_ACTION_TIMEOUT_S,
    MODEL_CAMERA_MAX_AGE_S,
    MODEL_MAX_TRACKING_DEG,
    MODEL_OBSERVATION_MAX_AGE_S,
    PanelController,
    PanelError,
    PanelServer,
    RobotDescription,
    build_parser,
    detect_port,
    run_command,
)
from soarm.panel_cameras import CameraManager

DT = 0.05
DESCRIPTION = RobotDescription.load(soarm.ROBOT_DIR)


def ticks(degrees: float) -> int:
    return round(REST_TICKS + degrees * 4096 / 360)


def torque(link: FakeBusLink) -> set[int]:
    return {registers[feetech.ADDR_TORQUE_ENABLE] for registers in link.registers.values()}


def connect(link: FakeBusLink, context: BackendContext) -> PanelController:
    bus = feetech.FeetechBus("COM_TEST")
    bus.open(link=link)
    arm = feetech.FeetechArm(context, bus=bus)
    arm.connect()
    return PanelController(arm, DESCRIPTION)


@pytest.fixture
def link() -> FakeBusLink:
    return FakeBusLink()


@pytest.fixture
def panel(link: FakeBusLink, so101_context) -> PanelController:
    return connect(link, so101_context())


def test_description(so101_context):
    assert DESCRIPTION.context("COM_TEST", {}) == so101_context()


def test_eeprom_narrows_the_limit(so101_context):
    link = FakeBusLink()
    link.registers[1][feetech.ADDR_MIN_ANGLE_LIMIT] = ticks(-45.0)
    link.registers[1][feetech.ADDR_MAX_ANGLE_LIMIT] = ticks(120.0)
    panel = connect(link, so101_context())
    assert panel.limits["shoulder_pan"] == (-45.0, 105.0)


def test_clip_notice(panel):
    panel.jog("shoulder_pan", 500.0)
    assert panel.target["shoulder_pan"] == 105.0
    assert panel.clip == "clipped to the limit: shoulder pan 500.0 -> 105.0"

    panel.jog("shoulder_pan", -10.0)
    assert panel.clip is None


def test_ramp(panel):
    panel.jog("shoulder_pan", 2.0)
    panel.tick(DT)
    assert panel.command["shoulder_pan"] == pytest.approx(DEFAULT_SPEED * DT)
    panel.tick(DT)
    assert panel.command["shoulder_pan"] == 2.0

    panel.jog("elbow_flex", 90.0)
    panel.tick(5.0)
    assert panel.command["elbow_flex"] == pytest.approx(DEFAULT_SPEED * MAX_DT_S)

    panel.set_speed(1000.0)
    panel.jog("wrist_flex", 90.0)
    panel.tick(DT)
    assert panel.command["wrist_flex"] == pytest.approx(DESCRIPTION.max_vel["wrist_flex"] * DT)


def test_stop(panel, link):
    panel.jog("shoulder_pan", 30.0)
    link.registers[1][feetech.ADDR_PRESENT_POSITION] = ticks(1.0)
    panel.tick(DT)

    panel.stop()
    assert panel.target == panel.measured
    assert panel.command == panel.measured

    # The arm coasts on after the last reading; the hold stays where STOP saw it.
    link.registers[1][feetech.ADDR_PRESENT_POSITION] = ticks(1.2)
    link.log.clear()
    panel.tick(DT)
    assert {value for _, dev_id, _, value in link.ops("sync") if dev_id == 1} == {ticks(1.0)}
    assert torque(link) == {1}


def test_torque_on_writes_the_goal_first(panel, link):
    """The goal register keeps where the servo was sent last: torque enabled first would jerk the arm."""
    panel.set_torque(False)
    link.registers[2][feetech.ADDR_PRESENT_POSITION] = ticks(-30.0)
    link.log.clear()

    panel.set_torque(True)

    goal = link.log.index(("sync", 2, feetech.ADDR_GOAL_POSITION, ticks(-30.0)))
    first_torque = link.log.index(("write", 1, feetech.ADDR_TORQUE_ENABLE, 1))
    assert goal < first_torque
    assert torque(link) == {1}


def test_torque_off(panel, link):
    panel.set_torque(False)
    assert torque(link) == {0}
    with pytest.raises(PanelError):
        panel.jog("shoulder_pan", 5.0)

    link.registers[1][feetech.ADDR_PRESENT_POSITION] = ticks(20.0)
    link.log.clear()
    panel.tick(DT)
    assert link.ops("sync") == []
    assert panel.target == panel.measured


def test_lost_page(panel, link):
    owner = panel.claim("Operator")
    panel.jog("shoulder_pan", 30.0)
    link.registers[1][feetech.ADDR_PRESENT_POSITION] = REST_TICKS + 3
    panel.last_heartbeat -= HEARTBEAT_TIMEOUT_S + 0.1
    panel.tick(DT)
    assert panel.target == panel.measured
    assert panel.command == panel.measured
    assert panel.state()["control"]["owner_id"] is None
    with pytest.raises(PanelError):
        panel.heartbeat(owner["token"])


def test_dead_link_holds_the_command(panel, link):
    panel.jog("shoulder_pan", 30.0)
    link.silent = True
    panel.tick(DT)
    assert panel.command["shoulder_pan"] == 0.0


def test_joint_outside_its_limit_is_not_snapped(so101_context):
    """An arm folded at rest can stand past robot.yaml's limit; clamping the first command would snap it."""
    link = FakeBusLink()
    link.registers[2][feetech.ADDR_PRESENT_POSITION] = ticks(-100.0)
    panel = connect(link, so101_context())

    panel.jog("shoulder_lift", 10.0)
    panel.tick(DT)
    assert link.registers[2][feetech.ADDR_GOAL_POSITION] == ticks(-100.0 + DEFAULT_SPEED * DT)


@pytest.fixture
def port(panel: PanelController) -> Iterator[int]:
    server = PanelServer(("127.0.0.1", 0), panel)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield server.server_port
    server.shutdown()
    server.server_close()


def call(port: int, method: str, path: str, body: object = None,
         content_type: str = "application/json") -> HTTPResponse:
    connection = HTTPConnection("127.0.0.1", port, timeout=5)
    connection.request(method, path, json.dumps(body) if body is not None else None,
                       {"Content-Type": content_type})
    return connection.getresponse()


def test_http_api(port, panel):
    assert call(port, "GET", "/").getheader("Location") == "http://127.0.0.1:8780/"

    panel.last_heartbeat = 0.0
    assert json.load(call(port, "GET", "/api/state"))["joints"] == list(DESCRIPTION.joints)
    assert panel.last_heartbeat == 0.0
    assert json.load(call(port, "GET", "/api/telemetry"))["joints"] == list(DESCRIPTION.joints)
    owner = json.load(call(port, "POST", "/api/control/claim", {
        "label": "Test operator", "emergency_stop_generation": 0,
    }))
    token = {"token": owner["token"]}

    assert call(port, "POST", "/api/jog", {**token, "joint": "elbow_flex", "delta": 5}).status == 200
    assert panel.target["elbow_flex"] == 5.0

    # A cross-site form or text/plain POST needs no CORS preflight; it must not move the arm.
    assert call(port, "POST", "/api/jog", {"joint": "elbow_flex", "delta": 5}, "text/plain").status == 415
    assert call(port, "POST", "/api/jog", {**token, "joint": "elbow", "delta": 5}).status == 400
    assert call(port, "POST", "/api/target", {
        **token, "joint": "elbow_flex", "value": float("nan"),
    }).status == 400
    assert call(port, "POST", "/api/dance", {}).status == 404
    assert panel.target["elbow_flex"] == 5.0


def test_two_clients_and_observers_do_not_keep_owner_alive(port, panel):
    owner = json.load(call(port, "POST", "/api/control/claim", {
        "label": "Alice", "emergency_stop_generation": 0,
    }))
    token = {"token": owner["token"]}
    assert call(port, "POST", "/api/control/claim", {
        "label": "Bob", "emergency_stop_generation": 0,
    }).status == 409
    for command, body in [("jog", {"joint": "elbow_flex", "delta": 10}),
                          ("target", {"joint": "elbow_flex", "value": 10}),
                          ("home", {}), ("torque", {"on": False}), ("speed", {"value": 50})]:
        assert call(port, "POST", "/api/" + command, body).status == 403
        assert call(port, "POST", "/api/" + command, {**body, "token": "wrong"}).status == 403
    assert call(port, "POST", "/api/jog", {**token, "joint": "elbow_flex", "delta": 10}).status == 200
    panel.last_heartbeat -= HEARTBEAT_TIMEOUT_S + 0.1
    expired_at = panel.last_heartbeat
    for _ in range(3):
        state = json.load(call(port, "GET", "/api/state"))
        assert owner["token"] not in json.dumps(state)
    assert panel.last_heartbeat == expired_at
    panel.tick(DT)
    assert panel.target == panel.measured
    assert panel.state()["control"]["owner_id"] is None
    assert call(port, "POST", "/api/control/heartbeat", token).status == 403
    other = json.load(call(port, "POST", "/api/control/claim", {
        "label": "Bob", "emergency_stop_generation": 0,
    }))
    assert other["owner_id"] != owner["owner_id"]
    assert call(port, "POST", "/api/control/release", token).status == 403
    assert call(port, "POST", "/api/control/release", {"token": other["token"]}).status == 200


def test_expiry_is_enforced_before_takeover_and_command(port, panel):
    owner = json.load(call(port, "POST", "/api/control/claim", {
        "label": "Alice", "emergency_stop_generation": 0,
    }))
    panel.jog("elbow_flex", 10)
    panel.last_heartbeat -= HEARTBEAT_TIMEOUT_S + 0.1
    other = json.load(call(port, "POST", "/api/control/claim", {
        "label": "Bob", "emergency_stop_generation": 0,
    }))
    assert other["token"] != owner["token"]
    assert panel.target == panel.measured
    panel.last_heartbeat -= HEARTBEAT_TIMEOUT_S + 0.1
    assert call(port, "POST", "/api/home", {"token": other["token"]}).status == 403


def test_stop_available_to_observer(port, panel, link):
    owner = panel.claim("Alice")
    panel.jog("elbow_flex", 10)
    assert call(port, "POST", "/api/stop", {}).status == 200
    assert panel.target == panel.measured
    assert torque(link) == {1}
    assert panel.state()["control"]["owner_id"] is None
    # A jog already queued by the old controlling browser cannot resume after STOP.
    assert call(port, "POST", "/api/jog", {
        "token": owner["token"], "joint": "elbow_flex", "delta": 10,
    }).status == 403
    assert panel.target == panel.measured


def test_stale_feedback_cannot_claim_or_renew(port, panel):
    panel._sample_at -= HEARTBEAT_TIMEOUT_S + 0.1
    assert call(port, "POST", "/api/control/claim", {
        "label": "Alice", "emergency_stop_generation": 0,
    }).status == 409
    panel.tick(DT)
    owner = json.load(call(port, "POST", "/api/control/claim", {
        "label": "Alice", "emergency_stop_generation": 0,
    }))
    panel.jog("elbow_flex", 20)
    panel._sample_at -= HEARTBEAT_TIMEOUT_S + 0.1
    heartbeat_before = panel.last_heartbeat
    sample_before = panel._sample_at
    state = json.load(call(port, "GET", "/api/state"))
    assert state["age_s"] > HEARTBEAT_TIMEOUT_S
    assert panel.last_heartbeat == heartbeat_before
    assert panel._sample_at == sample_before
    assert call(port, "POST", "/api/control/heartbeat", {"token": owner["token"]}).status == 409
    assert panel.state()["control"]["owner_id"] is None
    assert panel.target == panel.measured
    assert panel.last_heartbeat == heartbeat_before


def test_stale_feedback_rejects_movement(port, panel):
    owner = panel.claim("Alice")
    panel._sample_at -= HEARTBEAT_TIMEOUT_S + 0.1
    assert call(port, "POST", "/api/target", {
        "token": owner["token"], "joint": "elbow_flex", "value": 20,
    }).status == 409
    assert panel.target == panel.measured
    assert panel.state()["control"]["owner_id"] is None


def test_returning_loop_discards_target_before_ramping(panel, link):
    panel.claim("Alice")
    panel.jog("shoulder_pan", 30)
    panel._sample_at -= HEARTBEAT_TIMEOUT_S + 0.1
    # Fresh owner heartbeats cannot make an interrupted feedback loop safe to resume.
    link.registers[1][feetech.ADDR_PRESENT_POSITION] = ticks(5.0)
    link.log.clear()
    panel.tick(DT)
    assert panel.target == panel.measured
    assert panel.command == panel.measured
    assert panel.state()["control"]["owner_id"] is None
    assert panel.state()["age_s"] < HEARTBEAT_TIMEOUT_S
    assert {value for _, dev_id, _, value in link.ops("sync") if dev_id == 1} == {ticks(5.0)}


class _ViewerStub:
    def __init__(self):
        self.selected = None
        self.closed = False

    def status(self):
        return {"available": True, "port": 9876, "error": None, "selected_joint": self.selected}

    def select_joint(self, joint):
        self.selected = joint

    def close(self):
        self.closed = True


@pytest.fixture
def workbench(panel):
    def forbidden_capture(source):
        pytest.fail(f"HTTP tests must never open a camera: {source}")

    cameras = CameraManager(capture_factory=forbidden_capture)
    viewer = _ViewerStub()
    server = PanelServer(("127.0.0.1", 0), panel, cameras=cameras, viewer=viewer)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)
    assert viewer.closed


def test_observer_cached_state_includes_workbench_sources(workbench, panel, link):
    owner = panel.claim("Alice")
    heartbeat_before = panel.last_heartbeat
    sequence_before = panel.state()["sequence"]
    link.log.clear()
    for path in ("/api/state", "/api/telemetry", "/api/state"):
        state = json.load(call(workbench.server_port, "GET", path))
        assert state["sequence"] == sequence_before
        assert state["cameras"]["overhead"]["status"] == "unconfigured"
        assert state["cameras"]["wrist"]["source"] is None
        assert state["viewer"]["available"] is True
        assert state["viewer"]["port"] == 9876
        assert state["control"]["owner_id"] == owner["owner_id"]
        assert owner["token"] not in json.dumps(state)
    assert panel.last_heartbeat == heartbeat_before
    assert link.log == []


def test_camera_http_requires_owner_and_valid_distinct_devices(workbench, panel):
    port = workbench.server_port
    assert call(port, "POST", "/api/cameras", {"overhead": 0, "wrist": 1}).status in (400, 403)
    assert call(port, "POST", "/api/cameras", {
        "token": "wrong", "overhead": 0, "wrist": 1,
    }).status == 403
    assert workbench.cameras.status()["overhead"]["source"] is None
    owner = panel.claim("Alice")
    for overhead, wrist in [(0, 0), (-1, 1), (True, 1), ("0", 1), (0, 1.5)]:
        assert call(port, "POST", "/api/cameras", {
            "token": owner["token"], "overhead": overhead, "wrist": wrist,
        }).status == 400
    assert workbench.cameras.status()["overhead"]["source"] is None
    assert call(port, "POST", "/api/cameras", {
        "token": owner["token"], "overhead": 0, "wrist": 1,
    }).status == 200
    state = json.load(call(port, "GET", "/api/state"))
    assert state["cameras"]["overhead"]["source"] == 0
    assert state["cameras"]["wrist"]["source"] == 1


def test_workbench_redirects_browser_and_keeps_api(workbench):
    port = workbench.server_port
    workbench.dashboard_url = "https://dashboard.example/"
    response = call(port, "GET", "/")
    assert response.status == 307
    assert response.getheader("Location") == "https://dashboard.example/"
    assert response.getheader("Cache-Control") == "no-store"
    for path in ("/panel.js", "/panel.css"):
        assert call(port, "GET", path).status == 404
    for path in ("/api/state", "/api/telemetry"):
        capabilities = json.load(call(port, "GET", path))["capabilities"]
        assert {"/api/control/claim", "/api/emergency-stop", "/api/cartesian/start"} <= set(
            capabilities["operator_commands"]
        )
        assert capabilities["model_control"] is True
    assert call(port, "GET", "/cameras/overhead.jpg").status == 503
    assert call(port, "GET", "/cameras/wrist.jpg").status == 503


def test_failed_loop_clears_owner_and_old_target(panel, monkeypatch):
    owner = panel.claim("Alice")
    panel.jog("elbow_flex", 30)
    original = panel.arm.read_joints
    def fail():
        raise OSError("test link failure")
    monkeypatch.setattr(panel.arm, "read_joints", fail)
    with pytest.raises(OSError):
        panel.tick(DT)
    state = panel.state()
    assert not state["sample"]["loop_ok"]
    assert not state["sample"]["connected"]
    assert state["control"]["owner_id"] is None
    assert panel.target == panel.measured
    assert state["events"][-1]["level"] == "error"
    monkeypatch.setattr(panel.arm, "read_joints", original)
    panel.tick(DT)
    assert panel.target == panel.measured
    with pytest.raises(PanelError):
        panel.heartbeat(owner["token"])


def test_fault_clears_owner_and_target(panel, monkeypatch):
    panel.claim("Alice")
    panel.jog("elbow_flex", 30)
    monkeypatch.setattr(panel.arm, "faults", lambda: ["test servo fault"])
    panel.tick(DT)
    assert panel.state()["control"]["owner_id"] is None
    assert panel.target == panel.measured
    with pytest.raises(PanelError):
        panel.claim("Bob")


def test_close_before_start_preserves_torque_off(panel, link):
    panel.set_torque(False)
    panel.close()
    panel.close()
    assert torque(link) == {0}


def test_close_leaves_torque_on(panel, link):
    """What Ctrl-C does before FeetechArm.disconnect, which leaves torque on too."""
    panel.jog("shoulder_pan", 30.0)
    panel.tick(DT)

    panel.start()
    panel.close()
    assert link.registers[1][feetech.ADDR_GOAL_POSITION] == REST_TICKS
    assert torque(link) == {1}


def test_cli_defaults():
    args = build_parser().parse_args([])
    assert args.port is None
    assert args.listen == "127.0.0.1:8765"
    assert args.baudrate == 1_000_000
    assert args.robot == soarm.ROBOT_DIR
    assert not args.no_browser


class _Port:
    def __init__(self, device: str, vid: int | None):
        self.device = device
        self.vid = vid


def test_detect_port_picks_the_one_usb_serial_device():
    ports = [_Port("/dev/cu.Bluetooth", None), _Port("/dev/cu.usbmodem1", 0x1A86)]
    assert detect_port(ports) == "/dev/cu.usbmodem1"
    with pytest.raises(ValueError, match="No USB-serial device.*--port"):
        detect_port([_Port("/dev/cu.Bluetooth", None)])
    with pytest.raises(ValueError, match="Several USB-serial devices: COM5, COM7.*--port"):
        detect_port([_Port("COM7", 1), _Port("COM5", 1)])


@pytest.fixture
def model_session(panel, monkeypatch):
    cameras = {name: {"status": "live", "age_s": 0.01} for name in ("overhead", "wrist")}
    monkeypatch.setattr(panel, "_model_cameras", lambda: cameras)
    owner = panel.claim("Notebook operator")
    armed = run_command(panel, "model/arm", {"token": owner["token"], "duration_s": 10, "speed": 10})
    return {"token": owner["token"], "session_id": armed["session_id"],
            "observation_sequence": panel.state()["sequence"],
            "targets": {joint: 10.0 for joint in DESCRIPTION.joints}}, cameras


@pytest.mark.twin
def test_model_arming_seeds_real_supervisor_without_motion(panel, link, model_session):
    assert type(panel._model_supervisor).__module__ == "machinekind_twin.safety"
    assert panel._model_supervisor.command == panel.measured
    assert panel.target == panel.measured
    assert torque(link) == {1}
    body, _ = model_session
    link.log.clear()
    run_command(panel, "targets", body)
    assert link.log == []  # HTTP installs targets; only the control loop writes them.
    panel.tick(DT)
    for joint in DESCRIPTION.joints:
        assert abs(panel.command[joint] - panel.measured[joint]) <= 10 * DT + 0.09
    assert panel.target == body["targets"]


@pytest.mark.twin
@pytest.mark.parametrize("mutation", [
    {"gripper": float("nan")}, {"wrist_roll": float("inf")}, {"elbow_flex": True},
    {"shoulder_pan": 1e5}, {"unknown": 0},
])
def test_model_actions_validate_every_joint_before_mutating(panel, model_session, mutation):
    body, _ = model_session
    original = dict(panel.target)
    with pytest.raises(PanelError):
        run_command(panel, "targets", {**body, "targets": {**body["targets"], **mutation}})
    assert panel.target == original


@pytest.mark.twin
def test_model_actions_require_complete_vector_and_live_session(panel, model_session):
    body, _ = model_session
    for changes in ({"targets": {"shoulder_pan": 1}}, {"session_id": "old-session"},
                    {"token": "another-operator"}):
        with pytest.raises(PanelError):
            run_command(panel, "targets", {**body, **changes})
    assert panel.target == panel.measured


@pytest.mark.twin
@pytest.mark.parametrize("expired", ["duration", "actions", "owner"])
def test_model_watchdogs_revoke_despite_other_live_activity(panel, model_session, expired):
    body, _ = model_session
    run_command(panel, "targets", body)
    if expired == "duration":
        panel._model_deadline = 0.0
    elif expired == "actions":
        panel._model_last_action -= MODEL_ACTION_TIMEOUT_S + 0.01
        panel.heartbeat(body["token"])
    else:
        panel.last_heartbeat -= HEARTBEAT_TIMEOUT_S + 0.01
    panel.tick(DT)
    assert panel.target == panel.measured
    assert not panel.state()["model_session"]["active"]
    assert panel.state()["control"]["owner_id"] is None
    with pytest.raises(PanelError):
        run_command(panel, "targets", body)


@pytest.mark.twin
@pytest.mark.parametrize("camera", ["overhead", "wrist"])
@pytest.mark.parametrize("problem", ["missing", "stale", "invalid_age"])
def test_model_watchdog_requires_both_fresh_cameras(panel, model_session, camera, problem):
    body, cameras = model_session
    run_command(panel, "targets", body)
    if problem == "missing":
        del cameras[camera]
    elif problem == "stale":
        cameras[camera]["age_s"] = MODEL_CAMERA_MAX_AGE_S + 0.01
    else:
        cameras[camera]["age_s"] = float("nan")
    panel.tick(DT)
    assert panel.target == panel.measured
    assert panel.state()["control"]["owner_id"] is None
    assert not panel.state()["model_session"]["active"]


@pytest.mark.twin
@pytest.mark.parametrize("sequence", [True, -1, 1000000, "0", [], None])
def test_model_observation_must_be_a_known_sample(panel, model_session, sequence):
    body, _ = model_session
    with pytest.raises(PanelError, match="recent workbench observation"):
        run_command(panel, "targets", {**body, "observation_sequence": sequence})
    assert panel.state()["control"]["owner_id"] is None
    assert panel.target == panel.measured


@pytest.mark.twin
def test_model_observation_expiry_releases_control(panel, model_session):
    body, _ = model_session
    panel._sample_times[body["observation_sequence"]] -= MODEL_OBSERVATION_MAX_AGE_S + 0.01
    with pytest.raises(PanelError, match="recent workbench observation"):
        run_command(panel, "targets", body)
    assert not panel.state()["model_session"]["active"]
    assert panel.target == panel.measured


@pytest.mark.twin
def test_model_tracking_error_holds_measured_pose(panel, model_session):
    body, _ = model_session
    run_command(panel, "targets", body)
    panel.command["shoulder_pan"] = panel.measured["shoulder_pan"] + MODEL_MAX_TRACKING_DEG + 1
    panel.tick(DT)
    assert panel.target == panel.measured
    assert not panel.state()["model_session"]["active"]
    assert "tracking error" in panel.state()["events"][-1]["message"]


@pytest.mark.twin
def test_model_supervisor_does_not_advance_after_refused_write(panel, model_session, monkeypatch):
    body, _ = model_session
    run_command(panel, "targets", body)
    previous = dict(panel.command)
    monkeypatch.setattr(panel.arm, "send_joints", lambda goals: {})
    panel.tick(DT)
    assert panel.command == previous
    assert panel._model_supervisor.command == previous


@pytest.mark.twin
def test_model_disarm_retains_lease_but_stop_invalidates_old_actions(panel, model_session):
    body, _ = model_session
    run_command(panel, "targets", body)
    run_command(panel, "model/disarm", {"token": body["token"]})
    assert panel.target == panel.measured
    assert not panel.state()["model_session"]["active"]
    assert panel.state()["control"]["owner_id"] is not None
    new = run_command(panel, "model/arm", {"token": body["token"], "duration_s": 5, "speed": 5})
    assert new["session_id"] != body["session_id"]
    with pytest.raises(PanelError, match="current armed"):
        run_command(panel, "targets", body)
    panel.stop()
    assert not panel.state()["model_session"]["active"]
    with pytest.raises(PanelError):
        run_command(panel, "targets", {**body, "session_id": new["session_id"]})


@pytest.mark.twin
def test_model_http_capabilities_and_manual_exclusion(workbench, panel, monkeypatch):
    port = workbench.server_port
    state = json.load(call(port, "GET", "/api/state"))
    assert state["capabilities"]["model_control_version"] == 1
    assert not state["model_readiness"]["ready"]
    assert call(port, "POST", "/api/model/arm", {"duration_s": 10, "speed": 5}).status == 403
    owner = panel.claim("Notebook")
    token = {"token": owner["token"]}
    assert call(port, "POST", "/api/model/arm", {**token, "duration_s": 10, "speed": 5}).status == 409
    monkeypatch.setattr(workbench.cameras, "status", lambda: {
        name: {"status": "live", "age_s": 0.01} for name in ("overhead", "wrist")})
    # The controller callback was captured at server construction; supply the updated fake explicitly.
    panel._model_cameras = workbench.cameras.status
    assert json.load(call(port, "GET", "/api/state"))["model_readiness"]["ready"]
    for duration, speed in [(61, 5), (0, 5), (5, 16), (5, 0), (True, 5), (5, float("nan"))]:
        assert call(port, "POST", "/api/model/arm", {
            **token, "duration_s": duration, "speed": speed}).status == 400
    armed = json.load(call(port, "POST", "/api/model/arm", {**token, "duration_s": 10, "speed": 5}))
    assert "session_id" in armed
    assert call(port, "POST", "/api/targets", {
        **token, "session_id": armed["session_id"], "observation_sequence": panel.state()["sequence"],
        "targets": {joint: 10 for joint in DESCRIPTION.joints}}).status == 200
    for command, extra in [("target", {"joint": "shoulder_pan", "value": 0}),
                           ("home", {}), ("speed", {"value": 30}), ("torque", {"on": False}),
                           ("cameras", {"overhead": None, "wrist": None})]:
        assert call(port, "POST", "/api/" + command, {**token, **extra}).status == 409
    assert call(port, "POST", "/api/model/disarm", token).status == 200
    assert call(port, "POST", "/api/control/release", token).status == 200


@pytest.mark.twin
def test_model_arming_cannot_race_camera_reconfiguration(workbench, panel, monkeypatch):
    entered, finish = threading.Event(), threading.Event()
    original = workbench.cameras.configure

    def waiting_configure(overhead, wrist):
        entered.set()
        assert finish.wait(3)
        original(overhead, wrist)

    monkeypatch.setattr(workbench.cameras, "configure", waiting_configure)
    token = {"token": panel.claim("Operator")["token"]}
    results = []
    changing = threading.Thread(target=lambda: results.append(call(
        workbench.server_port, "POST", "/api/cameras", {**token, "overhead": 0, "wrist": 1}).status))
    changing.start()
    try:
        assert entered.wait(3)
        response = call(workbench.server_port, "POST", "/api/model/arm",
                        {**token, "duration_s": 10, "speed": 5})
        assert response.status == 409
        assert "reconfiguration" in json.load(response)["error"]
    finally:
        finish.set()
        changing.join(timeout=3)
    assert results == [200]
    assert not panel._camera_change_in_progress


@pytest.mark.twin
def test_model_arm_outside_limits_starts_at_measurement_then_ramps(so101_context):
    link = FakeBusLink()
    link.registers[2][feetech.ADDR_PRESENT_POSITION] = ticks(-100)
    panel = connect(link, so101_context())
    panel._model_cameras = lambda: {name: {"status": "live", "age_s": 0}
                                   for name in ("overhead", "wrist")}
    owner = panel.claim("Notebook")
    before = panel.measured["shoulder_lift"]
    session = run_command(panel, "model/arm", {"token": owner["token"], "duration_s": 10, "speed": 5})
    assert panel.command["shoulder_lift"] == before
    run_command(panel, "targets", {"token": owner["token"], "session_id": session["session_id"],
                                   "observation_sequence": 0,
                                   "targets": {joint: 0 for joint in DESCRIPTION.joints}})
    panel.tick(DT)
    assert before <= panel.command["shoulder_lift"] <= before + 5 * DT + 0.09
    assert panel.command["shoulder_lift"] < panel.limits["shoulder_lift"][0]


@pytest.mark.twin
def test_emergency_stop_latches_and_reset_rejects_delayed_model_action(workbench, panel, link, model_session):
    body, _ = model_session
    port = workbench.server_port
    run_command(panel, "targets", body)

    # An inference request already has a lease and session when another client stops it.
    latched = {"latched": True, "generation": 1, "hold_status": "sent", "hold_error": None}
    assert json.load(call(port, "POST", "/api/emergency-stop", {}))["emergency_stop"] == latched
    queued_reset = {"emergency_stop_generation": 1}
    latched["generation"] = 2
    assert json.load(call(port, "POST", "/api/emergency-stop", {}))["emergency_stop"] == latched
    assert call(port, "POST", "/api/emergency-stop/reset", queued_reset).status == 409
    state = json.load(call(port, "GET", "/api/state"))
    assert state["emergency_stop"] == latched
    assert not state["model_readiness"]["ready"]
    assert not state["model_session"]["active"]
    assert state["control"]["owner_id"] is None
    assert state["target"] == state["command"] == state["measured"]
    assert torque(link) == {1}
    for command, payload in (
        ("control/claim", {"label": "New operator"}),
        ("control/heartbeat", {"token": body["token"]}),
        ("model/arm", {"token": body["token"], "duration_s": 5, "speed": 5}),
        ("targets", body),
        ("home", {"token": body["token"]}),
        ("torque", {"token": body["token"], "on": False}),
    ):
        assert call(port, "POST", "/api/" + command, payload).status == 409
    panel.tick(DT)
    assert panel.state()["target"] == panel.state()["measured"]

    cleared = {"latched": False, "generation": 2, "hold_status": "not_requested", "hold_error": None}
    reset = {"emergency_stop_generation": 2}
    assert json.load(call(port, "POST", "/api/emergency-stop/reset", reset))["emergency_stop"] == cleared
    assert json.load(call(port, "POST", "/api/emergency-stop/reset", reset))["emergency_stop"] == cleared
    assert call(port, "POST", "/api/control/claim", {
        "label": "Old queued notebook", "emergency_stop_generation": 0,
    }).status == 409
    new_owner = json.load(call(port, "POST", "/api/control/claim", {
        "label": "New operator", "emergency_stop_generation": 2,
    }))
    assert call(port, "POST", "/api/targets", body).status == 403
    assert call(port, "POST", "/api/control/release", {"token": body["token"]}).status == 403
    assert panel.state()["control"]["owner_id"] == new_owner["owner_id"]
    assert panel.state()["target"] == panel.state()["measured"]

    assert call(port, "POST", "/api/emergency-stop", {}).status == 200
    assert call(port, "POST", "/api/emergency-stop/reset", reset).status == 409
    assert panel.state()["emergency_stop"]["generation"] == 3
    assert panel.state()["emergency_stop"]["latched"]


def test_emergency_hold_never_sends_cached_goal_on_feedback_recovery(panel, monkeypatch):
    owner = panel.claim("Operator")
    goals = []
    sent = {}

    def send(hold):
        goals.append(dict(hold))
        return sent

    monkeypatch.setattr(panel, "_send", send)
    panel.emergency_stop()
    assert panel.state()["emergency_stop"]["hold_status"] == "unconfirmed"
    assert panel.state()["emergency_stop"]["hold_error"]
    assert len(goals) == 1

    monkeypatch.setattr(panel.arm, "read_joints", lambda: dict(panel.measured))
    panel.arm._stale = ["shoulder_pan"]
    panel.tick(DT)
    assert len(goals) == 1
    assert panel.state()["emergency_stop"]["hold_status"] == "pending"

    fresh = {**panel.measured, "shoulder_pan": panel.measured["shoulder_pan"] + 5}
    monkeypatch.setattr(panel.arm, "read_joints", lambda: fresh)
    panel.arm._stale = []
    sent.update(fresh)
    panel.tick(DT)
    assert goals[-1] == fresh
    assert panel.state()["emergency_stop"]["hold_status"] == "sent"
    assert panel.state()["emergency_stop"]["hold_error"] is None
    with pytest.raises(PanelError, match="Emergency stop is latched"):
        panel.heartbeat(owner["token"])


def test_failed_stop_persistence_leaves_latch_and_prior_hardware_marker(panel, tmp_path, monkeypatch):
    from soarm.panel import EmergencyStopState

    path = tmp_path / "emergency-stop.json"
    state = EmergencyStopState(path=path, hardware_session=True)
    state.persist(latched=False)
    panel._emergency_stop = state

    def failed_write(**kwargs):
        raise OSError("read-only filesystem")

    monkeypatch.setattr(state, "persist", failed_write)
    with pytest.raises(PanelError, match="saving it failed"):
        panel.emergency_stop()
    assert panel.state()["emergency_stop"]["latched"]
    with pytest.raises(PanelError, match="Emergency stop is latched"):
        panel.claim("Another operator")
    assert EmergencyStopState.load(path).latched
