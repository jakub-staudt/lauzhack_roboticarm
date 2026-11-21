"""Wire DTOs for responses the dashboard builds itself. Proxied controller bodies have none."""

from enum import StrEnum

from pydantic import BaseModel


class HealthStatus(StrEnum):
    OK = 'ok'


class HealthResponse(BaseModel):
    """Liveness of the dashboard process only; says nothing about the controller behind it."""

    status: HealthStatus = HealthStatus.OK
