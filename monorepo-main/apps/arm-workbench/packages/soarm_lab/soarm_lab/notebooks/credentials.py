"""Explicit operator credentials for notebook inference."""

from __future__ import annotations

import getpass
import os
import stat
import warnings
from pathlib import Path


def codex_profile(profile: str | Path | None) -> Path:
    """Check an isolated file-auth profile without opening its credential file."""
    if not profile:
        raise RuntimeError("Set CODEX_PROFILE to your dedicated Codex profile after device login.")
    folder = Path(profile).expanduser().resolve()
    excluded = [(Path.home() / ".codex").resolve()]
    if os.environ.get("CODEX_HOME"):
        excluded.append(Path(os.environ["CODEX_HOME"]).expanduser().resolve())
    if any(folder.is_relative_to(path) for path in excluded):
        raise RuntimeError("Use a dedicated profile outside the default or inherited CODEX_HOME.")
    auth = folder / "auth.json"
    if not folder.is_dir() or not auth.is_file() or auth.is_symlink():
        raise RuntimeError("The dedicated profile needs its own file-based Codex device login.")
    if os.name == "posix":
        for path in (folder, auth):
            info = path.stat()
            if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) & 0o077:
                raise RuntimeError("Use a profile you own with mode 700 and auth.json mode 600.")
        if auth.stat().st_nlink != 1:
            raise RuntimeError("Use an independently created auth.json in the dedicated profile.")
    return folder


def require_dream_key(key: str | None) -> str:
    if not isinstance(key, str) or not key or any(char.isspace() for char in key):
        raise ValueError("Enter your Dream API key through the notebook's hidden prompt.")
    return key


def prompt_dream_key() -> str:
    """Require a hidden terminal or notebook prompt."""
    with warnings.catch_warnings():
        warnings.simplefilter("error", getpass.GetPassWarning)
        try:
            return require_dream_key(getpass.getpass("Your Dream API key: "))
        except (getpass.GetPassWarning, EOFError):
            raise RuntimeError("This kernel could not open a hidden credential prompt.") from None
