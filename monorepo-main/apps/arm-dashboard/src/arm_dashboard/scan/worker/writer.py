import json
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import TextIO

import cv2
import numpy as np

from arm_dashboard.core.typing.scan import ScanFiles
from arm_dashboard.scan.worker.constants import CollectionFiles
from arm_dashboard.scan.worker.exceptions import InvalidFrameError
from arm_dashboard.scan.worker.schemas import CollectorStatus, FrameRecord


class FrameLog:
    """Append-only ``frames.jsonl``, flushed per row so a killed collector loses nothing written."""

    def __init__(self, stream: TextIO) -> None:
        self._stream = stream

    def append(self, record: FrameRecord) -> None:
        self._stream.write(record.model_dump_json(exclude_unset=True) + '\n')
        self._stream.flush()


class ArtifactWriter:
    """Every file the collector produces in its session directory.

    Files the browser polls (status, preview images, mosaic) are replaced atomically.
    """

    def __init__(self, output_dir: Path) -> None:
        self._output_dir = output_dir

    def prepare(self) -> None:
        self._output_dir.mkdir(parents=True, exist_ok=True)

    def write_status(self, status: CollectorStatus) -> None:
        payload = json.dumps(status.model_dump(mode='json', exclude_none=True), indent=2) + '\n'
        self._replace(ScanFiles.STATUS, payload.encode())

    def write_raw_frame(self, filename: str, data: bytes) -> None:
        (self._output_dir / filename).write_bytes(data)

    def write_image(self, filename: str, image: np.ndarray) -> None:
        ok, encoded = cv2.imencode(Path(filename).suffix, image)
        if not ok:
            raise InvalidFrameError('Image encoding failed')
        self._replace(filename, encoded.tobytes())

    @contextmanager
    def frame_log(self) -> Iterator[FrameLog]:
        with (self._output_dir / CollectionFiles.FRAME_LOG).open('w') as stream:
            yield FrameLog(stream)

    def _replace(self, filename: str, data: bytes) -> None:
        target = self._output_dir / filename
        staged = target.with_suffix(CollectionFiles.STAGING_SUFFIX)
        staged.write_bytes(data)
        staged.replace(target)
