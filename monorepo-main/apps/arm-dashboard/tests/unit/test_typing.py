from pathlib import Path

import pytest
from pydantic import ValidationError

from arm_dashboard.core.typing import (
    CameraDelivery,
    CameraName,
    ControllerRoutes,
    ScanLimits,
    ScanPhase,
    ScanStatus,
    ScanUnit,
    ScanWorkerArgs,
    UpstreamResponse,
)


def delivery(available_fps: tuple[int, ...] = (2, 10, 15)) -> CameraDelivery:
    return CameraDelivery(
        default_fps=2, direct_default_fps=10, direct_hosts=('10.0.0.1', 'host'), available_fps=available_fps
    )


def test_camera_delivery_serializes_with_derived_fields() -> None:
    assert delivery().model_dump(mode='json') == {
        'default_fps': 2,
        'direct_default_fps': 10,
        'direct_hosts': ['10.0.0.1', 'host'],
        'available_fps': [2, 10, 15],
        'max_fps': 15,
        'selection_query': '?fps=<2|10|15>',
        'buffering': 'Each fresh snapshot is fetched after the previous part is written.',
        'latency_note': 'Output cadence is not a sensor-to-screen latency measurement.',
    }


def test_camera_delivery_derives_from_configured_fps() -> None:
    custom = delivery(available_fps=(2, 5))

    assert (custom.max_fps, custom.selection_query) == (5, '?fps=<2|5>')


@pytest.mark.parametrize('camera', list(CameraName))
def test_snapshot_route_per_camera(camera: CameraName) -> None:
    assert ControllerRoutes.snapshot(camera) == f'/cameras/{camera.value}.jpg'


@pytest.mark.parametrize(
    ('phase', 'active'), [(phase, phase in {'starting', 'collecting'}) for phase in ScanPhase]
)
def test_only_starting_and_collecting_are_active(phase: ScanPhase, active: bool) -> None:
    assert phase.is_active is active


def test_idle_status_serializes_exactly_the_fields_that_were_set() -> None:
    idle = ScanStatus(phase=ScanPhase.IDLE, captured_frames=0, units=ScanUnit.PIXELS, metric_calibrated=False)

    assert idle.model_dump(mode='json', exclude_unset=True) == {
        'phase': 'idle',
        'captured_frames': 0,
        'units': 'pixels',
        'metric_calibrated': False,
    }


def test_status_keeps_collector_fields_and_records_rewrites() -> None:
    written = {'phase': 'collecting', 'captured_frames': 3, 'coverage': {'observed_output_pixels': 7}}

    rewritten = ScanStatus.model_validate(written).model_copy(
        update={'phase': ScanPhase.INTERRUPTED, 'note': 'gone', 'session': 'scan-x'}
    )

    assert rewritten.model_dump(mode='json', exclude_unset=True) == written | {
        'phase': 'interrupted',
        'note': 'gone',
        'session': 'scan-x',
    }


def test_status_rejects_unknown_phase() -> None:
    with pytest.raises(ValidationError):
        ScanStatus.model_validate({'phase': 'paused'})


@pytest.mark.parametrize('seconds', [ScanLimits.DURATION_MIN_S - 0.5, ScanLimits.DURATION_MAX_S + 1])
def test_worker_args_reject_duration_outside_limits(seconds: float) -> None:
    with pytest.raises(ValidationError):
        ScanWorkerArgs(base_url='http://127.0.0.1:8765', output_dir=Path('/tmp/scan'), seconds=seconds)


def test_worker_args_accept_limits() -> None:
    for seconds in (ScanLimits.DURATION_MIN_S, ScanLimits.DURATION_MAX_S):
        args = ScanWorkerArgs(base_url='http://127.0.0.1:8765', output_dir=Path('/tmp/scan'), seconds=seconds)
        assert args.seconds == seconds


@pytest.mark.parametrize(
    'model',
    [
        delivery(),
        ScanStatus(phase=ScanPhase.IDLE),
        UpstreamResponse(status_code=200, content_type='application/json', body=b'{}'),
    ],
    ids=type,
)
def test_shared_shapes_are_immutable(model: object) -> None:
    field = next(iter(type(model).model_fields))  # type: ignore[attr-defined]

    with pytest.raises(ValidationError):
        setattr(model, field, getattr(model, field))
