"""Digital twin: the simulated arm follows the REAL arm.

Read-only: it only GETs the controller dashboard's /api/state (the measured joint angles) a few times a second and commands the
simulator's joints to the same angles. It never sends anything to the real arm. If the real arm stops answering the twin keeps its
last pose and reports how old that is.
"""

from __future__ import annotations

import http.client
import json
import threading
import time
from collections.abc import Callable
from urllib.parse import urlsplit

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def http_fetch(base_url: str, timeout_s: float = 3.0) -> Callable[[], dict]:
    """GET {base_url}/api/state over one kept-alive connection (a new connection per poll costs round trips on a relayed link)."""
    parts = urlsplit(base_url)
    local = threading.local()

    def fetch() -> dict:
        for attempt in (0, 1):
            conn = getattr(local, "conn", None)
            if conn is None:
                conn = local.conn = http.client.HTTPConnection(parts.hostname, parts.port or 80, timeout=timeout_s)
            try:
                conn.request("GET", "/api/state")
                response = conn.getresponse()
                data = response.read()
                if response.status != 200:
                    raise OSError(f"HTTP {response.status}")
                return json.loads(data)
            except (http.client.HTTPException, OSError, ValueError):
                conn.close()
                local.conn = None
                if attempt == 1:
                    raise
        raise OSError("unreachable")

    return fetch


class Twin:
    def __init__(self, fetch: Callable[[], dict], apply: Callable[[dict[str, float]], None], rate_hz: float = 8.0,
                 enabled: bool = True, clock: Callable[[], float] = time.monotonic) -> None:
        self._fetch, self._apply, self._period, self._clock = fetch, apply, 1.0 / rate_hz, clock
        self._enabled = enabled
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="twin")
        self._last_ok: float | None = None
        self._error = ""
        self._real: dict = {}

    # ------------------------------------------------------------------ control
    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stopping.set()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, on: bool) -> None:
        self._enabled = bool(on)

    def poll_once(self) -> bool:
        """One read of the real arm and, when enabled, one command to the simulated one. Returns True if the read worked."""
        try:
            state = self._fetch()
            measured = {j: float(state["measured"][j]) for j in JOINTS}
        except Exception as exc:                                   # noqa: BLE001 - any failure just marks the data as stale
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"[:160]
            return False
        with self._lock:
            self._last_ok, self._error = self._clock(), ""
            self._real = {"joints": measured, "connected": bool(state.get("connected")), "torque": bool(state.get("torque")),
                          "emergency_stop": bool((state.get("emergency_stop") or {}).get("latched")),
                          "owner": (state.get("control") or {}).get("label")}
        if self._enabled:
            self._apply(measured)
        return True

    def status(self) -> dict:
        with self._lock:
            age = None if self._last_ok is None else self._clock() - self._last_ok
            return {"enabled": self._enabled, "ok": age is not None and age < 3 * self._period + 1.0, "age_s": None if age is None else round(age, 2),
                    "error": self._error, "real": dict(self._real)}

    def _run(self) -> None:
        while not self._stopping.is_set():
            started = self._clock()
            self.poll_once()
            self._stopping.wait(max(0.0, self._period - (self._clock() - started)))
