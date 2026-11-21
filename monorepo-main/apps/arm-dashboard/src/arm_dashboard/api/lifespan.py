from collections.abc import AsyncGenerator, Callable
from contextlib import AbstractAsyncContextManager, asynccontextmanager

from fastapi import FastAPI

from arm_dashboard.api.constants import AppStateKeys
from arm_dashboard.api.v1.dependencies import ServicesFactory, open_upstream_clients
from arm_dashboard.core.settings import Settings

Lifespan = Callable[[FastAPI], AbstractAsyncContextManager[None]]


def build_lifespan(settings: Settings, services_factory: ServicesFactory) -> Lifespan:
    """Startup: open upstream clients, assemble services, adopt the newest scan session.
    Shutdown: stop the collector, then close the clients."""

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncGenerator[None]:
        async with open_upstream_clients(settings) as clients:
            services = services_factory(settings, clients)
            await services.scan.restore()
            setattr(app.state, AppStateKeys.SERVICES, services)
            try:
                yield
            finally:
                await services.scan.shutdown()

    return lifespan
