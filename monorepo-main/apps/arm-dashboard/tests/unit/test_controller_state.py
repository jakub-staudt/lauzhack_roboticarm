import json

import httpx
import pytest

from arm_dashboard.controller import ControllerClient, ControllerStateService, InvalidControllerStateError
from arm_dashboard.core.typing import CameraDelivery, ControllerCommand, UpstreamResponse

pytestmark = pytest.mark.anyio

DELIVERY = CameraDelivery(
    default_fps=2, direct_default_fps=10, direct_hosts=('100.85.56.14', 'host'), available_fps=(2, 10, 15)
)


def service_answering(status: int, body: bytes) -> tuple[ControllerStateService, list[str]]:
    paths: list[str] = []

    def handler(request: httpx.Request) -> httpx.Response:
        paths.append(request.url.path)
        return httpx.Response(status, headers={'Content-Type': 'application/json'}, content=body)

    http = httpx.AsyncClient(transport=httpx.MockTransport(handler), base_url='http://controller.test')
    return ControllerStateService(ControllerClient(http), DELIVERY), paths


def as_json(payload: object) -> bytes:
    return json.dumps(payload).encode()


async def test_state_gets_camera_delivery_and_viewer_proxy_url() -> None:
    service, paths = service_answering(200, as_json({'connected': True, 'viewer': {'port': 8766}}))

    state = await service.fetch()

    assert state == {
        'connected': True,
        'viewer': {'port': 8766, 'proxy_url': '/viewer/'},
        'camera_delivery': DELIVERY.model_dump(mode='json'),
        'operator_commands': [command.value for command in ControllerCommand],
    }
    assert paths == ['/api/state']


@pytest.mark.parametrize(
    'state',
    [{'connected': False}, {'viewer': None}, {'viewer': 'disabled'}],
    ids=['no-viewer', 'null-viewer', 'string-viewer'],
)
async def test_viewer_is_only_touched_when_it_is_an_object(state: dict[str, object]) -> None:
    service, _ = service_answering(200, as_json(state))

    enriched = await service.fetch()

    assert isinstance(enriched, dict)
    dashboard_fields = {'camera_delivery', 'operator_commands'}
    preserved = {key: value for key, value in enriched.items() if key not in dashboard_fields}
    assert preserved == state


@pytest.mark.parametrize(
    ('advertised', 'expected'),
    [
        (['/api/home', '/api/flag-wave/start', '/api/targets', '/api/home'], ['/api/home']),
        ([], []),
        (None, []),
    ],
)
async def test_operator_commands_use_both_services(advertised: object, expected: list[str]) -> None:
    capabilities = {'operator_commands': advertised, 'model_control': True}
    service, _ = service_answering(200, as_json({'capabilities': capabilities}))

    state = await service.fetch()

    assert isinstance(state, dict)
    assert state['capabilities'] == capabilities
    assert state['operator_commands'] == expected


async def test_each_fetch_gets_a_fresh_delivery_copy() -> None:
    service, _ = service_answering(200, as_json({}))

    first = await service.fetch()
    assert isinstance(first, dict)
    first['camera_delivery']['default_fps'] = 99

    second = await service.fetch()
    assert isinstance(second, dict)
    assert second['camera_delivery']['default_fps'] == 2


async def test_non_200_is_returned_untouched() -> None:
    body = as_json({'error': 'starting'})
    service, _ = service_answering(503, body)

    assert await service.fetch() == UpstreamResponse(
        status_code=503, content_type='application/json', body=body
    )


@pytest.mark.parametrize('body', [b'not json', b'\xff\xfe', b'[1, 2, 3]', b'"text"', b'null'])
async def test_unusable_200_is_an_invalid_state(body: bytes) -> None:
    service, _ = service_answering(200, body)

    with pytest.raises(InvalidControllerStateError):
        await service.fetch()
