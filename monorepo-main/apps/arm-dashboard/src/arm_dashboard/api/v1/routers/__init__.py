from fastapi import APIRouter

from arm_dashboard.api.v1.routers import cameras, controller, health, pages, scan, viewer

# Mounted without a prefix: the paths (/api/state, /cameras/...) are fixed by the
# workbench frontend in arm-workbench, so `v1` versions this delivery layer, not the URLs.
router = APIRouter()
for _module in (health, pages, controller, cameras, viewer, scan):
    router.include_router(_module.router)

__all__ = ['router']
