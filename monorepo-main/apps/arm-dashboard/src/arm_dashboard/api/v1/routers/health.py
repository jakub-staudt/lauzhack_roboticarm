from typing import Final, final

from fastapi import APIRouter

from arm_dashboard.api.v1.schemas import HealthResponse


@final
class HealthRoutes:
    # Used by scripts/deploy.sh and, later, the Docker health check.
    LIVENESS: Final = '/healthz'


router = APIRouter(tags=['health'])


@router.get(HealthRoutes.LIVENESS, response_model=HealthResponse)
async def health() -> HealthResponse:
    return HealthResponse()
