from arm_dashboard.api.middlewares.origin import SameOriginMiddleware
from arm_dashboard.api.middlewares.registry import install_middlewares
from arm_dashboard.api.middlewares.rules import Decision, same_origin

__all__ = ['Decision', 'SameOriginMiddleware', 'install_middlewares', 'same_origin']
