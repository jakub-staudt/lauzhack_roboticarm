from fastapi import FastAPI

from arm_dashboard.api.middlewares.origin import SameOriginMiddleware


def install_middlewares(app: FastAPI) -> None:
    """Install the stack innermost-first; each later ``add_middleware`` wraps the earlier ones.

    1. ``SameOriginMiddleware`` — directly around routing: refuses cross-origin POST/WebSocket
       before any route or handler runs. Future auth goes outside it.
    """
    app.add_middleware(SameOriginMiddleware)
