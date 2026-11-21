"""Shared test root. ``tests/`` is on the import path (``pythonpath`` in pyproject.toml), so the
fakes import as ``fakes``.

Async tests use anyio's pytest plugin (shipped with anyio, a Starlette dependency), marked with
``@pytest.mark.anyio``, and run on asyncio only: uvicorn and production run on asyncio.
"""

import pytest


@pytest.fixture
def anyio_backend() -> str:
    return 'asyncio'
