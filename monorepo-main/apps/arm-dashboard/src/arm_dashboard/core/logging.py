"""Process-wide logging. Called once from the entrypoint, never from ``create_app``:
reconfiguring the root logger inside tests would detach pytest's log capture.
"""

import logging.config
from typing import Final, final

from arm_dashboard.core.settings import LogLevel


@final
class LogConfig:
    FORMAT: Final = '%(asctime)s %(levelname)s %(name)s: %(message)s'
    # Uvicorn's own loggers propagate to the root handler so every line shares one format.
    UVICORN_LOGGERS: Final = ('uvicorn', 'uvicorn.error', 'uvicorn.access')
    # httpx logs every request at INFO: tens of lines a second with state polling and MJPEG.
    # Upstream failures are logged by our own clients with method and path instead.
    QUIET_LOGGERS: Final = ('httpx', 'httpcore')
    QUIET_LEVEL: Final = 'WARNING'


def setup_logging(level: LogLevel) -> None:
    logging.config.dictConfig(
        {
            'version': 1,
            'disable_existing_loggers': False,
            'formatters': {'default': {'format': LogConfig.FORMAT}},
            'handlers': {'stderr': {'class': 'logging.StreamHandler', 'formatter': 'default'}},
            'root': {'level': level.value, 'handlers': ['stderr']},
            'loggers': {
                **{name: {'handlers': [], 'propagate': True} for name in LogConfig.UVICORN_LOGGERS},
                **{name: {'level': LogConfig.QUIET_LEVEL} for name in LogConfig.QUIET_LOGGERS},
            },
        }
    )
