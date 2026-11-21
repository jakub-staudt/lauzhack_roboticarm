"""Cached measurements and read-only twin pose rendering, without a servo bus."""
from __future__ import annotations

import copy
import math
import socket
from types import SimpleNamespace

import pytest

from soarm import ROBOT_DIR
from soarm.panel_viewer import STALE_AFTER_S, PoseViewer, measured_pose

pytestmark = pytest.mark.twin


def snapshot():
    return {"measured": {"shoulder_pan": 90.0, "shoulder_lift": -20.0, "elbow_flex": 30.0,
                         "wrist_flex": 10.0, "wrist_roll": -45.0, "gripper": 50.0},
            "target": {"shoulder_pan": -90.0},
            "age_s": 0.01, "loop_ok": True, "connected": True}


def test_measured_pose_ignores_targets_and_copies_cache():
    state = snapshot()
    before = copy.deepcopy(state)
    pose, stale = measured_pose(state, tuple(state["measured"]))
    assert pose["shoulder_pan"] == 90.0
    assert stale is False
    pose["shoulder_pan"] = 0
    assert state == before


@pytest.mark.parametrize("change", [dict(age_s=STALE_AFTER_S + 0.1), dict(connected=False),
                                     dict(loop_ok=False), dict(age_s=float("nan")), dict(age_s=None)])
def test_stale_feed_never_claims_live(change):
    state = {**snapshot(), **change}
    assert measured_pose(state, tuple(state["measured"]))[1] is True


@pytest.mark.parametrize("value", [None, True, float("nan"), float("inf"), "90"])
def test_incomplete_measurements_do_not_fall_back_to_home_or_target(value):
    state = snapshot()
    state["measured"]["shoulder_pan"] = value
    with pytest.raises(ValueError, match="shoulder_pan"):
        measured_pose(state, tuple(state["measured"]))


def test_nested_sample_cache_is_supported():
    state = snapshot()
    state["sample"] = {key: state.pop(key) for key in ("age_s", "loop_ok", "connected")}
    assert measured_pose(state, tuple(state["measured"]))[1] is False


def test_selection_without_renderer_only_validates_names():
    viewer = PoseViewer(ROBOT_DIR, lambda: pytest.fail("selection must not query state"), "127.0.0.1")
    viewer.select_joint("elbow_flex")
    assert viewer.status()["selected_joint"] == "elbow_flex"
    with pytest.raises(ValueError, match="Unknown joint"):
        viewer.select_joint("not_a_joint")
    assert viewer.status()["selected_joint"] == "elbow_flex"
    viewer.close()


def test_unavailable_renderer_is_reported_and_cleaned_up(monkeypatch):
    viewer = PoseViewer(ROBOT_DIR, lambda: {}, "127.0.0.1")
    stopped = []
    viewer._server = SimpleNamespace(stop=lambda: stopped.append(True))
    def unavailable():
        raise ImportError("optional twin missing")
    monkeypatch.setattr(viewer, "_initialize", unavailable)
    viewer.start()
    assert viewer.status()["available"] is False
    assert "optional twin missing" in viewer.status()["error"]
    assert stopped == [True]
    viewer.close()


def test_real_twin_mapping_highlight_and_cache_only():
    pytest.importorskip("machinekind_twin")
    pytest.importorskip("viser")
    state = snapshot()
    before = copy.deepcopy(state)
    calls = []
    def cached_state():
        calls.append(True)
        return state
    # Pick a free loopback port; no hardware or simulation backend is constructed.
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    viewer = PoseViewer(ROBOT_DIR, cached_state, "127.0.0.1", port,
                        gripper_range_q=(-0.1, 0.9))
    try:
        viewer._initialize()
        assert viewer._root.visible is False
        viewer.select_joint("elbow_flex")
        viewer._update()
        kin = viewer._kin
        q = kin.data.qpos[kin.qadr]
        assert q[0] == pytest.approx(math.pi / 2)
        assert q[-1] == pytest.approx(0.4)
        assert viewer._marker.visible is True
        joint = kin.model.joint("elbow_flex").id
        assert viewer._marker.position == pytest.approx(kin.data.xanchor[joint])
        assert viewer.status()["stale"] is False
        state["connected"] = False
        viewer._update()
        assert viewer.status()["stale"] is True
        assert "STALE" in viewer._label.text
        assert len(calls) == 2
        state["connected"] = True
        assert state == before
    finally:
        viewer.close()


def test_each_connected_client_receives_a_fresh_panel_minimize():
    minimized = []
    viewer = PoseViewer(ROBOT_DIR, lambda: pytest.fail("UI setup must not query state"), "127.0.0.1")
    client = SimpleNamespace(gui=SimpleNamespace(main_panel=SimpleNamespace(
        minimize=lambda: minimized.append(True))))
    viewer._configure_client(client)
    viewer._configure_client(client)
    assert minimized == [True, True]


def test_named_environment_is_static_visible_without_a_measured_pose(monkeypatch):
    pytest.importorskip("machinekind_twin")
    pytest.importorskip("viser")
    from soarm.panel_environment import STANDARD_LAYOUT
    monkeypatch.setenv("RAMION_WORKSPACE_LAYOUT", STANDARD_LAYOUT)
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    viewer = PoseViewer(ROBOT_DIR, lambda: {}, "127.0.0.1", port)
    try:
        viewer._initialize()
        assert viewer._root.visible is False  # no invented robot pose
        assert len(viewer._environment_handles) == 5
        assert all(handle.visible for handle in viewer._environment_handles)
        assert viewer.status()["environment"]["table_z_m"] == pytest.approx(-.0074)
        assert viewer.status()["environment"]["visual_only"] is True
        # Geometry is not inserted into the kinematics model or its world frame.
        import mujoco
        baseline = mujoco.MjModel.from_xml_path(str(ROBOT_DIR / "so101.xml"))
        assert viewer._kin.model.ngeom == baseline.ngeom
        assert viewer._kin.model.body_pos == pytest.approx(baseline.body_pos)
        assert viewer._kin.model.jnt_pos == pytest.approx(baseline.jnt_pos)
    finally:
        viewer.close()
