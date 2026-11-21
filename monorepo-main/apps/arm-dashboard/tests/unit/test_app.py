from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager

import httpx
import pytest

from arm_dashboard.core.settings import ServerSettings, Settings
from arm_dashboard.main import create_app

pytestmark = pytest.mark.anyio

BASE_URL = 'http://dashboard.test'


@asynccontextmanager
async def client(*, docs_enabled: bool = False) -> AsyncGenerator[httpx.AsyncClient]:
    app = create_app(Settings(server=ServerSettings(docs_enabled=docs_enabled)))
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url=BASE_URL) as http:
        yield http


async def test_healthz_reports_ok() -> None:
    async with client() as http:
        reply = await http.get('/healthz')

    assert reply.status_code == 200
    assert reply.json() == {'status': 'ok'}


async def test_docs_are_hidden_by_default() -> None:
    async with client() as http:
        assert (await http.get('/docs')).status_code == 404
        assert (await http.get('/openapi.json')).status_code == 404


async def test_docs_can_be_enabled() -> None:
    async with client(docs_enabled=True) as http:
        assert (await http.get('/openapi.json')).status_code == 200


async def test_trailing_slash_is_not_redirected() -> None:
    async with client() as http:
        assert (await http.get('/healthz/', follow_redirects=False)).status_code == 404
