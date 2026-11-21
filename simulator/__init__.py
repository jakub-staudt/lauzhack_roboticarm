"""SO-101 gaze station simulator.

Needs only `pip install -r simulator/requirements.txt` (mujoco, numpy, pyyaml): the two workbench packages it uses
(`soarm`, `soarm_lab`) are imported straight from this checkout, so no ROS and no `uv sync` are required.
"""

import sys
from pathlib import Path

_PACKAGES = Path(__file__).resolve().parents[1] / "monorepo-main" / "apps" / "arm-workbench" / "packages"
for _name in ("soarm", "soarm_lab"):
    _path = str(_PACKAGES / _name)
    if (_PACKAGES / _name).is_dir() and _path not in sys.path:
        sys.path.append(_path)  # appended, so an installed copy of the packages still wins
