"""Transport vocabulary of the dashboard API: state keys, headers, media types, page files."""

from typing import Final, final


@final
class AppStateKeys:
    """What the lifespan publishes on ``app.state`` for dependencies to read."""

    SERVICES: Final = 'services'


@final
class Headers:
    CACHE_CONTROL: Final = 'Cache-Control'
    CONNECTION: Final = 'Connection'
    ORIGIN: Final = 'origin'
    HOST: Final = 'host'
    UPGRADE: Final = 'upgrade'
    CONTENT_LENGTH: Final = 'content-length'
    WS_KEY: Final = 'sec-websocket-key'
    WS_VERSION: Final = 'sec-websocket-version'
    WS_PROTOCOL: Final = 'sec-websocket-protocol'


@final
class HeaderValues:
    NO_STORE: Final = 'no-store'
    CLOSE: Final = 'close'
    WEBSOCKET: Final = 'websocket'


@final
class MediaTypes:
    HTML: Final = 'text/html; charset=utf-8'
    JAVASCRIPT: Final = 'text/javascript; charset=utf-8'
    CSS: Final = 'text/css; charset=utf-8'
    JPEG: Final = 'image/jpeg'


@final
class ResponseHeaders:
    """Every dashboard answer must stay out of caches: state, frames and pages change constantly."""

    NO_STORE: Final = {Headers.CACHE_CONTROL: HeaderValues.NO_STORE}
    # Relayed and streamed answers end their connection, as the legacy proxy did.
    NO_STORE_CLOSE: Final = {
        Headers.CACHE_CONTROL: HeaderValues.NO_STORE,
        Headers.CONNECTION: HeaderValues.CLOSE,
    }


@final
class LocalPages:
    """Pages served from ``settings.server.static_dir``: URL paths → (file, media type)."""

    BY_PATH: Final = {
        '/': ('panel.html', MediaTypes.HTML),
        '/panel.css': ('panel.css', MediaTypes.CSS),
        '/panel.js': ('panel.js', MediaTypes.JAVASCRIPT),
        '/map': ('scan.html', MediaTypes.HTML),
        '/map/': ('scan.html', MediaTypes.HTML),
        '/scan': ('scan.html', MediaTypes.HTML),
        '/scan/': ('scan.html', MediaTypes.HTML),
        '/map/manual': ('board-map.html', MediaTypes.HTML),
        '/board-map.js': ('board-map.js', MediaTypes.JAVASCRIPT),
    }


@final
class ScanRoutes:
    STATUS: Final = '/api/scan/status'
    START: Final = '/api/scan/start'
    STOP: Final = '/api/scan/stop'
    IMAGE_TEMPLATE: Final = '/api/scan/images/{artifact}'


@final
class CameraRoutes:
    SNAPSHOT_TEMPLATE: Final = '/cameras/{camera}.jpg'
    STREAM_TEMPLATE: Final = '/cameras/{camera}.mjpg'


@final
class ViewerPaths:
    """The viewer answers with and without the trailing slash; neither redirects."""

    ALL: Final = ('/viewer', '/viewer/')
