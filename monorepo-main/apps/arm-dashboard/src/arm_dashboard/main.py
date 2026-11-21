"""Application assembly. Nothing here but wiring: settings → lifespan → middleware → errors → routers."""

from typing import Final, final

import uvicorn
from fastapi import FastAPI

from arm_dashboard.api.errors import install_error_handlers
from arm_dashboard.api.lifespan import build_lifespan
from arm_dashboard.api.middlewares import install_middlewares
from arm_dashboard.api.v1 import router as v1_router
from arm_dashboard.api.v1.dependencies import ServicesFactory, build_services
from arm_dashboard.core.logging import setup_logging
from arm_dashboard.core.settings import Settings, get_settings


@final
class AppMeta:
    TITLE: Final = 'Ramion arm dashboard'
    DOCS_PATH: Final = '/docs'
    OPENAPI_PATH: Final = '/openapi.json'


def create_app(
    settings: Settings | None = None, *, services_factory: ServicesFactory = build_services
) -> FastAPI:
    settings = settings or get_settings()
    docs_enabled = settings.server.docs_enabled
    app = FastAPI(
        title=AppMeta.TITLE,
        docs_url=AppMeta.DOCS_PATH if docs_enabled else None,
        redoc_url=None,
        openapi_url=AppMeta.OPENAPI_PATH if docs_enabled else None,
        # The legacy server never redirected `/map/manual/` to `/map/manual`; it answered 404.
        redirect_slashes=False,
        lifespan=build_lifespan(settings, services_factory),
    )
    install_middlewares(app)
    install_error_handlers(app)
    app.include_router(v1_router)
    return app


def run() -> None:
    """Console entrypoint. One process only: scan session state lives in this process's memory."""
    settings = get_settings()
    setup_logging(settings.server.log_level)
    # log_config=None: uvicorn keeps the configuration above instead of installing its own.
    uvicorn.run(create_app(settings), host=settings.server.host, port=settings.server.port, log_config=None)
