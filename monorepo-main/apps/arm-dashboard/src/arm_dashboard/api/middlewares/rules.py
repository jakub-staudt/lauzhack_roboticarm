"""Cross-cutting rules, transport-agnostic: they return a decision and raise nothing.

Phase 5 (auth, reservations) adds its rules here, next to same-origin.
"""

from enum import StrEnum


class Decision(StrEnum):
    ALLOW = 'allow'
    DENY = 'deny'


def same_origin(origin: str | None, host: str | None) -> Decision:
    """A browser request is allowed only from a page this dashboard served.

    No ``Origin`` header (curl, scripts, scan worker): allowed, as in the legacy proxy. With one,
    it must be exactly ``http://<Host>`` — the scheme the dashboard is served with.
    """
    if origin is None:
        return Decision.ALLOW
    return Decision.ALLOW if origin == f'http://{host or ""}' else Decision.DENY
