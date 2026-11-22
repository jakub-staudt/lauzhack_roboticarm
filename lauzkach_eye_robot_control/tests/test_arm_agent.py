"""The arm agent over real HTTP, against a FAKE controller. Nothing touches the real arm. Needs the robotic_arm environment."""

import threading
import time
from http.server import ThreadingHTTPServer

import pytest

pytest.importorskip("soarm_lab")
from tests.test_realarm import CAL, KIN, POINTS, FakeDashboard, corner  # noqa: E402

from lauzkach_eye_robot_control.arm_agent import AgentState, make_handler  # noqa: E402
from lauzkach_eye_robot_control.drive import RemoteRealClient  # noqa: E402
from lauzkach_eye_robot_control.realarm import RealArmClient, RealSettings  # noqa: E402

pytestmark = pytest.mark.skipif(POINTS is None, reason="needs the recorded calibration points")
TOKEN = "t" * 24


@pytest.fixture
def rig():
    dash = FakeDashboard()
    client = RealArmClient(dash, KIN, CAL, z_table=-0.003, settings=RealSettings(pause_s=0.0))
    agent = AgentState(client, TOKEN, deadman_s=0.8)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(agent))
    stop = threading.Event()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=agent.watchdog, args=(stop,), daemon=True).start()
    remote = RemoteRealClient(f"http://127.0.0.1:{server.server_address[1]}", TOKEN)
    yield dash, client, remote, server
    stop.set()
    server.shutdown()


def wait_done(remote, timeout=60.0):
    end = time.time() + timeout
    while time.time() < end:
        if remote.state()["task"]["state"] not in ("running", "planning"):
            return remote.state()["task"]["state"]
        time.sleep(0.05)
    raise AssertionError("move did not finish")


def test_one_go_command_runs_the_whole_move_next_to_the_controller(rig):
    dash, client, remote, _ = rig
    x, y = corner(0)
    assert remote.go_to_base(x, y, 3.0)["state"] == "planning"
    assert wait_done(remote) == "done"
    tip = remote.state()["tip_base"]
    assert abs(tip["x"] - x) < 0.004 and abs(tip["y"] - y) < 0.004 and abs(tip["z"] - 0.027) < 0.004
    assert dash.owner is None and len(dash.targets) > 100                           # many joint steps, driven by the agent


def test_a_wrong_or_missing_token_is_refused_and_moves_nothing(rig):
    dash, _, remote, server = rig
    bad = RemoteRealClient(remote.url, "x" * 24)
    with pytest.raises(RuntimeError, match="bad token"):
        bad.go_to_base(*corner(0), 3.0)
    with pytest.raises(RuntimeError, match="bad token"):
        bad.state()
    assert dash.targets == [] and ("POST", "/api/control/claim") not in dash.calls


def test_the_agents_refusals_reach_the_caller_as_plain_reasons(rig):
    dash, _, remote, _ = rig
    with pytest.raises(RuntimeError, match="at least 3 cm"):
        remote.go_to_base(*corner(0), 1.0)
    dash.m["gripper"] = 80.0
    with pytest.raises(RuntimeError, match="gripper"):
        remote.go_to_base(*corner(0), 3.0)
    assert dash.targets == []


def test_stop_halts_the_move_and_releases_control(rig):
    dash, client, remote, _ = rig
    client.settings.pause_s = client.mover.pause_s = 0.2
    remote.go_to_base(*corner(2), 3.0)
    time.sleep(0.6)
    remote.stop()
    assert wait_done(remote) == "stopped" and dash.stops >= 1 and dash.owner is None
    n = len(dash.targets)
    time.sleep(0.4)
    assert len(dash.targets) == n


def test_the_agent_holds_the_arm_if_the_laptop_goes_silent_during_a_move(rig):
    dash, client, remote, _ = rig
    client.settings.pause_s = client.mover.pause_s = 0.3
    remote.go_to_base(*corner(2), 3.0)                                              # then nobody asks for the state any more
    time.sleep(2.0)
    assert dash.stops >= 1 and dash.owner is None
    assert not client.mover.busy()


def test_an_unreachable_agent_is_a_clear_error_not_a_hang():
    with pytest.raises(RuntimeError, match="not reachable"):
        RemoteRealClient("http://127.0.0.1:9", TOKEN, timeout_s=1).state()
