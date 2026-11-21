"""The browser side of the relay, as the relay needs it. The API layer adapts Starlette's
``WebSocket`` to this, so the viewer package never imports a web framework.
"""

from typing import Protocol

from pydantic import BaseModel, ConfigDict


class SocketClosed(BaseModel):
    """The browser closed its socket with this code (1005 when it gave none)."""

    model_config = ConfigDict(frozen=True)

    code: int


ClientMessage = str | bytes | SocketClosed


class ClientSocketProtocol(Protocol):
    async def receive_message(self) -> ClientMessage:
        """The next text or binary message, or ``SocketClosed`` once the browser is gone."""
        ...

    async def send_message(self, message: str | bytes) -> None:
        """Send one message. Raises ``ConnectionError`` if the browser is gone."""
        ...

    async def close(self, code: int) -> None:
        """Close with ``code``. Idempotent, and a no-op once the browser is gone."""
        ...
