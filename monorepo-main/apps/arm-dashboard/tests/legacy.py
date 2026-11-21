"""Access to the legacy stdlib implementation in ``legacy/``, for parity checks only.

Deleted together with ``legacy/``.
"""

import importlib
import sys
from pathlib import Path
from types import ModuleType

APP_DIR = Path(__file__).resolve().parents[1]
LEGACY_DIR = APP_DIR / 'legacy'
STATIC_DIR = APP_DIR / 'src' / 'arm_dashboard' / 'static'


def load(name: str) -> ModuleType:
    """Import a legacy module (``server``, ``viewer_proxy``, ``scan_manager``, ``scan_worker``).

    The legacy modules import each other by bare name, so the legacy directory goes on sys.path.
    """
    if str(LEGACY_DIR) not in sys.path:
        sys.path.insert(0, str(LEGACY_DIR))
    return importlib.import_module(name)
