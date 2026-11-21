"""One automatic, read-only collection session: the scenario, step by step.

Every second: fetch the controller state and both camera snapshots concurrently, log each
capture, register it against the overhead anchor, refresh the candidate mosaic, write the
status. It reads only the controller's cached HTTP endpoints: no camera device, no serial
port, no motion command, no ownership token on disk.
"""

import json
import logging
import threading
import time
from collections.abc import Callable
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any

import cv2
import httpx
import numpy as np

from arm_dashboard.core.typing.camera import CameraName
from arm_dashboard.core.typing.controller import ControllerRoutes
from arm_dashboard.core.typing.scan import ScanPhase, ScanWorkerArgs
from arm_dashboard.scan.worker.constants import CollectionFiles, CollectionSettings
from arm_dashboard.scan.worker.exceptions import CollectorError
from arm_dashboard.scan.worker.fetcher import Fetched, FrameFetcher
from arm_dashboard.scan.worker.frames import FrameDecoder
from arm_dashboard.scan.worker.mosaic import Layer, MosaicRenderer
from arm_dashboard.scan.worker.registration import PlaneRegistrationProcessor
from arm_dashboard.scan.worker.schemas import (
    CollectorStatus,
    FrameRecord,
    RegistrationReason,
    RegistrationResult,
)
from arm_dashboard.scan.worker.writer import ArtifactWriter

logger = logging.getLogger(__name__)

Clock = Callable[[], float]

# One failed capture is recorded in its row and counted; the session goes on.
CAPTURE_ERRORS = (CollectorError, httpx.HTTPError, OSError, ValueError, cv2.error)


class ScanCollectionOrchestrator:
    def __init__(
        self,
        fetcher: FrameFetcher,
        decoder: FrameDecoder,
        registration: PlaneRegistrationProcessor,
        mosaic: MosaicRenderer,
        writer: ArtifactWriter,
        *,
        interval_s: float = CollectionSettings.INTERVAL_S,
        clock: Clock = time.monotonic,
    ) -> None:
        self._fetcher = fetcher
        self._decoder = decoder
        self._registration = registration
        self._mosaic = mosaic
        self._writer = writer
        self._interval_s = interval_s
        self._clock = clock

    def run(self, args: ScanWorkerArgs, stop: threading.Event) -> CollectorStatus:
        self._writer.prepare()
        session = _Session(self._clock(), args.seconds)
        self._writer.write_status(session.status)
        with self._writer.frame_log() as log, ThreadPoolExecutor(CollectionSettings.FETCH_WORKERS) as pool:
            cycle = 0
            while session.elapsed(self._clock()) < args.seconds and not stop.is_set():
                tick = self._clock()
                state_job = pool.submit(self._fetcher.fetch, ControllerRoutes.STATE)
                camera_jobs = {
                    camera: pool.submit(self._fetcher.fetch, ControllerRoutes.snapshot(camera))
                    for camera in CollectionSettings.CAMERAS
                }
                controller = _controller_snapshot(state_job)
                for camera, job in camera_jobs.items():
                    log.append(self._capture(session, cycle, camera, job, controller))
                self._refresh_mosaic(session)
                session.status.elapsed_s = round(
                    session.elapsed(self._clock()), CollectionSettings.STATUS_ELAPSED_DIGITS
                )
                session.status.capture_cycles = cycle + 1
                self._writer.write_status(session.status)
                cycle += 1
                stop.wait(max(0.0, self._interval_s - (self._clock() - tick)))
        session.finish(stopped=stop.is_set(), elapsed_s=session.elapsed(self._clock()))
        self._writer.write_status(session.status)
        logger.info(
            'Collection %s after %s capture cycles', session.status.phase, session.status.capture_cycles
        )
        return session.status

    def _capture(
        self,
        session: '_Session',
        cycle: int,
        camera: CameraName,
        job: 'Future[Fetched]',
        controller: dict[str, Any],
    ) -> FrameRecord:
        row: dict[str, Any] = {
            'index': cycle,
            'camera': camera,
            'elapsed_s': round(session.elapsed(self._clock()), CollectionSettings.ROW_ELAPSED_DIGITS),
            'controller': controller,
            'synchronized_exposure': False,
        }
        try:
            fetched = job.result()
            frame = self._decoder.decode(fetched.data)
            filename = CollectionFiles.RAW_FRAME_TEMPLATE.format(index=cycle, camera=camera.value)
            self._writer.write_raw_frame(filename, fetched.data)
            self._writer.write_image(f'{camera.value}.jpg', frame.image)
            row |= {
                'file': filename,
                'sha256': frame.sha256,
                'request_ms': fetched.request_ms,
                'width': frame.width,
                'height': frame.height,
                'sharpness': frame.sharpness,
            }
            session.status.captured_frames += 1
            registration = self._register(session, camera, frame.sha256, frame.image)
            if registration is not None:
                row['registration'] = registration
            session.seen.add((camera, frame.sha256))
        except CAPTURE_ERRORS as exc:
            row['error'] = str(exc)
            session.status.errors += 1
            logger.warning('Capture %s/%s failed: %s', cycle, camera, exc)
        return FrameRecord.model_validate(row)

    def _register(
        self, session: '_Session', camera: CameraName, digest: str, image: np.ndarray
    ) -> RegistrationResult | None:
        """Decide what this capture contributes; ``None`` for a wrist frame before any anchor."""
        if (camera, digest) in session.seen:
            session.status.duplicate_frames += 1
            return RegistrationResult(accepted=False, reason=RegistrationReason.IDENTICAL_CACHED_FRAME)
        if session.anchor is None:
            if camera != CollectionSettings.ANCHOR_CAMERA:
                return None
            session.anchor = image.copy()
            session.status.registered_frames += 1
            return RegistrationResult(accepted=True, reason=RegistrationReason.ANCHOR_VIEW)
        result = self._registration.register(session.anchor, image)
        homography = result.homography
        if homography is None:
            session.status.rejected_frames += 1
            return result
        session.status.registered_frames += 1
        if self._mosaic.should_add_layer(session.layers, session.anchor, image, homography):
            session.layers.append((image.copy(), homography))
        return result

    def _refresh_mosaic(self, session: '_Session') -> None:
        if session.anchor is None:
            return
        try:
            mosaic, coverage = self._mosaic.render(session.anchor, session.layers)
        except CollectorError as exc:
            session.status.mosaic_error = str(exc)
            return
        self._writer.write_image('mosaic.jpg', mosaic)
        session.status.coverage = coverage
        session.status.candidate_mosaic_layers = len(session.layers) + 1


class _Session:
    """Mutable state of one run: the status being reported and what has been seen so far."""

    def __init__(self, started: float, duration_s: float) -> None:
        self._started = started
        self.status = CollectorStatus(
            phase=ScanPhase.COLLECTING,
            started_utc=time.strftime(CollectionFiles.STARTED_UTC_FORMAT, time.gmtime()),
            duration_s=duration_s,
        )
        self.anchor: np.ndarray | None = None
        self.layers: list[Layer] = []
        self.seen: set[tuple[CameraName, str]] = set()

    def elapsed(self, now: float) -> float:
        return now - self._started

    def finish(self, *, stopped: bool, elapsed_s: float) -> None:
        self.status.phase = ScanPhase.STOPPED if stopped else ScanPhase.COMPLETE
        self.status.elapsed_s = round(elapsed_s, CollectionSettings.STATUS_ELAPSED_DIGITS)
        if not self.status.captured_frames:
            self.status.phase = ScanPhase.ERROR


def _controller_snapshot(job: 'Future[Fetched]') -> dict[str, Any]:
    """The logged subset of the controller state, or the reason it is missing."""
    try:
        fetched = job.result()
        raw = json.loads(fetched.data)
    except CAPTURE_ERRORS as exc:
        return {'error': str(exc)}
    if not isinstance(raw, dict):
        return {'error': 'Controller state is not a JSON object'}
    return {field: raw.get(field) for field in CollectionSettings.STATE_FIELDS} | {
        'request_ms': fetched.request_ms
    }
