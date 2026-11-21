import asyncio
import sys
from collections.abc import Callable
from pathlib import Path

import pytest
from pydantic import AnyHttpUrl

from arm_dashboard.core.typing import ScanWorkerArgs
from arm_dashboard.scan import ScanRuntimeMissingError, ScanWorkerRunner
from arm_dashboard.scan.runner import worker_argv

pytestmark = pytest.mark.anyio

WAIT_S = 5.0
# Stand-in collectors: print the arguments, then wait to be stopped.
OBEDIENT = 'import sys, time; print(sys.argv[1:], flush=True); time.sleep(30)'
STUBBORN = (
    'import signal, sys, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
    'print("ready", flush=True); time.sleep(30)'
)


def args(tmp_path: Path) -> ScanWorkerArgs:
    return ScanWorkerArgs(base_url=AnyHttpUrl('http://127.0.0.1:8765'), output_dir=tmp_path, seconds=60)


def runner_for(script: str) -> ScanWorkerRunner:
    return ScanWorkerRunner(command=(sys.executable, '-c', script), runtime_installed=lambda: True)


async def eventually(condition: Callable[[], object], what: str) -> None:
    try:
        async with asyncio.timeout(WAIT_S):
            while not condition():
                await asyncio.sleep(0.01)
    except TimeoutError:
        raise AssertionError(f'timed out waiting for {what}') from None


def test_worker_arguments_match_the_shared_flags(tmp_path: Path) -> None:
    assert worker_argv(args(tmp_path)) == [
        '--base',
        'http://127.0.0.1:8765',
        '--output',
        str(tmp_path),
        '--seconds',
        '60.0',
    ]


def test_missing_runtime_is_reported_with_the_install_command() -> None:
    runner = ScanWorkerRunner(runtime_installed=lambda: False)

    with pytest.raises(ScanRuntimeMissingError, match=r'\[scan\]'):
        runner.ensure_runtime()


def test_nothing_has_run_yet() -> None:
    runner = runner_for(OBEDIENT)

    assert (runner.has_started, runner.is_running, runner.has_exited) == (False, False, False)


async def test_start_logs_output_and_terminate_stops_it(tmp_path: Path) -> None:
    runner = runner_for(OBEDIENT)
    log = tmp_path / 'collector.log'

    await runner.start(args(tmp_path), log)
    await eventually(lambda: log.read_text(), 'collector output')

    assert runner.is_running
    assert "'--output', '" + str(tmp_path) + "'" in log.read_text()

    runner.terminate()
    await eventually(lambda: runner.has_exited, 'collector exit')
    assert (runner.has_started, runner.is_running) == (True, False)


async def test_terminate_without_a_collector_is_a_no_op() -> None:
    runner_for(OBEDIENT).terminate()


async def test_shutdown_kills_a_collector_that_ignores_sigterm(tmp_path: Path) -> None:
    runner = runner_for(STUBBORN)
    log = tmp_path / 'collector.log'
    await runner.start(args(tmp_path), log)
    await eventually(lambda: 'ready' in log.read_text(), 'collector ready')

    await runner.shutdown(timeout_s=0.2)

    assert runner.has_exited
