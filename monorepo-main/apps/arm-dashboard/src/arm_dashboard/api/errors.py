"""Domain exceptions → HTTP status, in one place.

Mapped by base class, so a new subclass needs nothing here.
"""

import logging
from http import HTTPStatus

from fastapi import FastAPI, Request
from fastapi.exception_handlers import http_exception_handler
from fastapi.responses import JSONResponse, Response
from starlette.exceptions import HTTPException as StarletteHTTPException

from arm_dashboard.core.exceptions import (
    ConflictError,
    DashboardError,
    InvalidRequestError,
    NotFoundError,
    UpstreamError,
)

logger = logging.getLogger(__name__)


class RequestBodyTooLargeError(DashboardError):
    """A command body over the configured limit. Raised before the controller is contacted."""


STATUS_BY_ERROR: tuple[tuple[type[DashboardError], HTTPStatus], ...] = (
    (RequestBodyTooLargeError, HTTPStatus.REQUEST_ENTITY_TOO_LARGE),
    (InvalidRequestError, HTTPStatus.BAD_REQUEST),
    (NotFoundError, HTTPStatus.NOT_FOUND),
    (ConflictError, HTTPStatus.CONFLICT),
    (UpstreamError, HTTPStatus.BAD_GATEWAY),
)


def status_for(exc: DashboardError) -> HTTPStatus:
    """The HTTP status of a domain error; also used for WebSocket denial responses."""
    for error_type, status in STATUS_BY_ERROR:
        if isinstance(exc, error_type):
            return status
    return HTTPStatus.INTERNAL_SERVER_ERROR


def error_body(exc: DashboardError) -> dict[str, str]:
    # scan.html shows `error` of a 409 to the operator; other answers keep FastAPI's `detail`.
    key = 'error' if isinstance(exc, ConflictError) else 'detail'
    return {key: str(exc)}


def error_response(exc: DashboardError) -> JSONResponse:
    """The HTTP answer to a domain error; also the body of a refused WebSocket upgrade."""
    status = status_for(exc)
    if status == HTTPStatus.INTERNAL_SERVER_ERROR:
        logger.error('Unmapped dashboard error', exc_info=exc)
    return JSONResponse(error_body(exc), status_code=status)


async def _dashboard_error(_request: Request, exc: Exception) -> Response:
    assert isinstance(exc, DashboardError)
    return error_response(exc)


async def _http_error(request: Request, exc: Exception) -> Response:
    assert isinstance(exc, StarletteHTTPException)
    # The legacy server answered 404 for a known path with the wrong method; keep that contract.
    if exc.status_code == HTTPStatus.METHOD_NOT_ALLOWED:
        exc = StarletteHTTPException(status_code=HTTPStatus.NOT_FOUND)
    return await http_exception_handler(request, exc)


def install_error_handlers(app: FastAPI) -> None:
    app.add_exception_handler(DashboardError, _dashboard_error)
    app.add_exception_handler(StarletteHTTPException, _http_error)
