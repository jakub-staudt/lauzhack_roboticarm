"""The start-up calibration: measure for a few seconds, lock the rectangle, then only watch for drift."""

import time
from pathlib import Path

import cv2
import numpy as np
import pytest

from lauzkach_eye_robot_control.camera import FileSource
from lauzkach_eye_robot_control.detect import detect
from lauzkach_eye_robot_control.server import DRIFT_FRAMES, Analyzer

FRAME = Path(__file__).parent / "fixtures" / "overhead_1.jpg"


@pytest.fixture()
def make():
    """Build and start analyzers; stop them all at the end so no thread is left running inside OpenCV."""
    started = []

    def build(path, calibration_s, min_frames) -> Analyzer:
        analyzer = Analyzer(FileSource(str(path), interval=0.05), calibration_s=calibration_s, min_frames=min_frames)
        analyzer.start()
        started.append(analyzer)
        return analyzer

    yield build
    for analyzer in started:
        analyzer.stop()


def wait_for(analyzer: Analyzer, state: str, timeout: float = 8.0) -> dict:
    end = time.time() + timeout
    while time.time() < end:
        snap = analyzer.snapshot()
        if snap["calibration"]["state"] == state:
            return snap
        time.sleep(0.05)
    raise AssertionError(f"calibration never reached {state!r}: {analyzer.snapshot()['calibration']}")


def test_locks_after_the_countdown_and_the_rectangle_stops_moving(make):
    analyzer = make(FRAME, 0.6, 4)
    first = wait_for(analyzer, "locked")
    cal = first["calibration"]
    expected = np.array(detect(cv2.imread(str(FRAME)))["polygon"])
    assert np.abs(np.array(cal["polygon"]) - expected).max() < 3
    assert "estimated" in cal["method"] and cal["n_frames"] >= 4 and cal["spread_px"] < 1.0
    assert first["locked"] and first["polygon_shown"] == cal["polygon"]
    time.sleep(0.3)
    later = analyzer.snapshot()
    assert later["seq"] > first["seq"] and later["polygon_shown"] == first["polygon_shown"]   # locked: identical on every frame


def test_no_rectangle_means_the_calibration_fails_with_a_reason(make, tmp_path):
    blank = tmp_path / "blank.png"
    cv2.imwrite(str(blank), np.full((480, 640, 3), 235, np.uint8))
    analyzer = make(blank, 0.5, 4)
    cal = wait_for(analyzer, "failed")["calibration"]
    assert "0 usable" in cal["reason"] and "tags" in cal["reason"]


def test_recalibrate_starts_a_new_countdown(make):
    analyzer = make(FRAME, 0.5, 3)
    wait_for(analyzer, "locked")
    analyzer.recalibrate()
    snap = analyzer.snapshot()
    assert snap["calibration"]["state"] == "waiting"
    wait_for(analyzer, "countdown")
    wait_for(analyzer, "locked")


def test_drift_warns_only_when_the_tags_stay_moved(make):
    analyzer = make(FRAME, 0.5, 3)
    wait_for(analyzer, "locked")
    base = detect(cv2.imread(str(FRAME)))

    def shifted(dx: float) -> dict:
        moved = {**base, "tags": [{**t, "center": [t["center"][0] + dx, t["center"][1]]} for t in base["tags"]],
                 "polygon": [[x + dx, y] for x, y in base["polygon"]]}
        return moved

    with analyzer.lock:
        for _ in range(DRIFT_FRAMES + 2):
            analyzer._check_drift(shifted(0.5))
        assert not analyzer.cal["drift_warn"]                 # sub-pixel wobble is normal
        for _ in range(DRIFT_FRAMES - 1):
            analyzer._check_drift(shifted(12))
        assert not analyzer.cal["drift_warn"]                 # one or two odd frames are not enough
        analyzer._check_drift(shifted(12))
        assert analyzer.cal["drift_warn"] and analyzer.cal["drift_px"] > 10
        analyzer._check_drift(shifted(0))                     # back where it was: the warning clears
        assert not analyzer.cal["drift_warn"]


def test_lock_uses_the_printed_left_line_when_the_arm_is_out_of_the_way(make):
    away = Path(__file__).parent / "fixtures" / "overhead_arm_away.jpg"
    cal = wait_for(make(away, 0.6, 4), "locked")["calibration"]
    assert cal["method"] == "left line seen and used"          # not the 1-2 cm worse estimate
    tl, _tr, _br, bl = np.array(cal["polygon"])
    assert np.abs(tl - (146, 120)).max() < 8 and np.abs(bl - (128, 348)).max() < 10
