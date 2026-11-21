"""Viser viewer vocabulary shared by the controller proxy (state enrichment) and the viewer relay."""

from typing import Final, final

from pydantic import BaseModel, ConfigDict


@final
class ViewerRoutes:
    """Where the viewer is reachable through the dashboard. Injected into state as ``viewer.proxy_url``."""

    PROXY_URL: Final = '/viewer/'


class ViewerHandshake(BaseModel):
    """The WebSocket upgrade headers the relay checks before contacting Viser.

    Same-origin is not here: it is one policy for POST commands and upgrades, enforced by the
    API middleware.
    """

    model_config = ConfigDict(frozen=True)

    key: str
    version: str | None
    protocol: str
    connection_tokens: frozenset[str]
