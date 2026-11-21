"""Collector entrypoint: ``python -m arm_dashboard.scan.worker --base URL --output DIR --seconds N``.

The Handler of this process: parse and validate the arguments, assemble the orchestrator, run it,
and turn SIGTERM/SIGINT into a graceful stop that still writes the final status.
"""

import argparse
import signal
import threading
from collections.abc import Sequence
from pathlib import Path

import cv2
import httpx
from pydantic import AnyHttpUrl

from arm_dashboard.core.logging import setup_logging
from arm_dashboard.core.settings import LogLevel
from arm_dashboard.core.typing.scan import ScanWorkerArgs, ScanWorkerFlags
from arm_dashboard.scan.worker.collector import ScanCollectionOrchestrator
from arm_dashboard.scan.worker.constants import CollectionSettings
from arm_dashboard.scan.worker.fetcher import FrameFetcher
from arm_dashboard.scan.worker.frames import FrameDecoder
from arm_dashboard.scan.worker.mosaic import MosaicRenderer
from arm_dashboard.scan.worker.registration import PlaneRegistrationProcessor
from arm_dashboard.scan.worker.writer import ArtifactWriter

DEFAULT_SECONDS = 60.0


def parse_args(argv: Sequence[str] | None = None) -> ScanWorkerArgs:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(ScanWorkerFlags.BASE_URL, dest='base_url', required=True)
    parser.add_argument(ScanWorkerFlags.OUTPUT_DIR, dest='output_dir', type=Path, required=True)
    parser.add_argument(ScanWorkerFlags.SECONDS, dest='seconds', type=float, default=DEFAULT_SECONDS)
    namespace = parser.parse_args(argv)
    return ScanWorkerArgs(
        base_url=AnyHttpUrl(namespace.base_url), output_dir=namespace.output_dir, seconds=namespace.seconds
    )


def collect(args: ScanWorkerArgs, stop: threading.Event) -> None:
    # trust_env=False: never route controller traffic through an HTTP(S)_PROXY from the environment.
    with httpx.Client(
        base_url=str(args.base_url), timeout=CollectionSettings.FETCH_TIMEOUT_S, trust_env=False
    ) as http:
        orchestrator = ScanCollectionOrchestrator(
            FrameFetcher(http),
            FrameDecoder(),
            PlaneRegistrationProcessor(),
            MosaicRenderer(),
            ArtifactWriter(args.output_dir),
        )
        orchestrator.run(args, stop)


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    setup_logging(LogLevel.INFO)
    # One OpenCV thread: the collector must not compete with the web server and the arm controller.
    cv2.setNumThreads(1)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    collect(args, stop)


if __name__ == '__main__':
    main()
