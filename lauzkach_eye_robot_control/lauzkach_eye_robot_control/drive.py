"""Turn a click on the camera picture into a move, and protect those moves against lag. Pure logic, no web server.

Everything here is meant to run NEXT TO THE ARM (on the server) later, with the browser only sending small intents:

* `CommandGuard`: every move command carries an id, the time it was sent and the "generation" the page last saw.
  A command that arrives too late (lag spike), twice (retry) or after a Stop is not executed. A Stop is always accepted.
* `DeadMan`: while a move runs, the page must stay in touch; if it vanishes the arm is held where it is.
* picture <-> (u, v) <-> arm coordinates, using the camera calibration and the robot's recorded points.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass

from lauzkach_eye_robot_control.projection import pixel_from_table, table_from_pixel

FACES = ("up", "right", "down", "left")      # which way the robot faces in the camera picture


# ---------------------------------------------------------------------------------------------- picture <-> (u, v)
def image_unit_to_uv(xu: float, yu: float, faces: str) -> tuple[float, float]:
    """Unit coordinates in the picture's rectangle (x right, y down, 0..1) -> (u, v): u to the robot's right, v away from it."""
    return {"up": (xu, 1 - yu), "right": (yu, xu), "down": (1 - xu, yu), "left": (1 - yu, 1 - xu)}[faces]


def uv_to_image_unit(u: float, v: float, faces: str) -> tuple[float, float]:
    return {"up": (u, 1 - v), "right": (v, u), "down": (1 - u, v), "left": (1 - v, 1 - u)}[faces]


def px_to_uv(px_to_cm: list[list[float]], px: float, py: float, faces: str, side_cm: float = 20.0, height_cm: float = 0.0,
             lift: list[float] | None = None) -> tuple[float, float]:
    """Picture pixel -> (u, v) using the camera's pixel -> table-cm matrix of the detected rectangle (0..side_cm).
    `height_cm` (and `lift`, see projection.py) correct for a point above the table."""
    x_cm, y_cm = table_from_pixel(px_to_cm, px, py, height_cm, lift)
    return image_unit_to_uv(x_cm / side_cm, y_cm / side_cm, faces)


def uv_to_px(px_to_cm: list[list[float]], u: float, v: float, faces: str, side_cm: float = 20.0, height_cm: float = 0.0,
             lift: list[float] | None = None) -> tuple[float, float]:
    xu, yu = uv_to_image_unit(u, v, faces)
    return pixel_from_table(px_to_cm, xu * side_cm, yu * side_cm, height_cm, lift)


# ---------------------------------------------------------------------------------------------- lag protection
@dataclass
class Admission:
    status: str                    # ok | duplicate | stale | old_generation | invalid
    detail: str = ""
    cached: dict | None = None     # for a duplicate: what the first copy returned

    @property
    def ok(self) -> bool:
        return self.status == "ok"


class CommandGuard:
    """Decides whether a move command may run. A command is {cid, sent_ms (server clock), gen, max_age_ms}."""

    def __init__(self, max_age_s: float = 2.0, history: int = 64, clock: Callable[[], float] = time.time, strict: bool = True) -> None:
        self.max_age_s, self.strict, self._clock = max_age_s, strict, clock
        self._lock = threading.Lock()
        self._seen: OrderedDict[str, dict | None] = OrderedDict()
        self._history = history
        self.generation = 0

    def now_ms(self) -> float:
        return self._clock() * 1000.0

    def admit(self, cid: str | None, sent_ms: float | None, generation: int | None, max_age_ms: float | None = None) -> Admission:
        if self.strict and (not cid or sent_ms is None or generation is None):
            return Admission("invalid", "a move command needs an id, the time it was sent and the generation the page last saw")
        with self._lock:
            if cid in self._seen:
                return Admission("duplicate", "this command was already received", self._seen[cid])
            if generation is not None and generation != self.generation:
                return Admission("old_generation", f"the arm was stopped since this was sent (generation {generation}, now {self.generation})")
            if sent_ms is not None:
                age = (self.now_ms() - sent_ms) / 1000.0
                limit = (max_age_ms / 1000.0) if max_age_ms else self.max_age_s
                if age > limit:
                    return Admission("stale", f"the command arrived {age:.1f} s after it was sent (limit {limit:.1f} s): the link is lagging")
                if age < -1.0:
                    return Admission("invalid", "the page's clock is ahead of the server's; re-sync the clock")
            if cid:
                self._seen[cid] = None
                while len(self._seen) > self._history:
                    self._seen.popitem(last=False)
        return Admission("ok")

    def record(self, cid: str | None, result: dict) -> None:
        """Remember what a command returned, so a retried copy gets the same answer instead of moving the arm again."""
        if cid:
            with self._lock:
                self._seen[cid] = result

    def stop(self) -> int:
        """A Stop: everything sent before it, and still on its way, is void from now on."""
        with self._lock:
            self.generation += 1
            return self.generation


class DeadMan:
    """Calls `on_expire` once if the page has been silent for `timeout_s` while `is_active()` (a move is running)."""

    def __init__(self, timeout_s: float, is_active: Callable[[], bool], on_expire: Callable[[], None],
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.timeout_s, self._active, self._expire, self._clock = timeout_s, is_active, on_expire, clock
        self._last = clock()
        self._fired = False
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="deadman")

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=2)

    def touch(self) -> None:
        self._last, self._fired = self._clock(), False

    def check(self) -> bool:
        """One check; True if it fired. (The thread calls this; tests call it directly.)"""
        if not self._fired and self._clock() - self._last > self.timeout_s and self._active():
            self._fired = True
            self._expire()
            return True
        return False

    def _run(self) -> None:
        while not self._stopping.wait(0.25):
            self.check()


# ---------------------------------------------------------------------------------------------- the simulator
class SimClient:
    """HTTP client for the simulator's web server (it plans and executes whole moves on its side)."""

    def __init__(self, url: str, timeout_s: float = 4.0) -> None:
        if not url.startswith("http://"):
            raise ValueError("the simulator URL must be http://host:port")
        self.url, self.timeout_s = url.rstrip("/"), timeout_s

    def _call(self, path: str, body: dict | None = None) -> dict:
        request = urllib.request.Request(self.url + path, data=None if body is None else json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return json.loads(response.read())
        except urllib.error.HTTPError as exc:
            try:
                message = json.loads(exc.read()).get("error", "")
            except ValueError:
                message = ""
            raise SimError(message or f"the simulator answered HTTP {exc.code}", exc.code) from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise SimError(f"the simulator is not reachable at {self.url} ({type(exc).__name__})", 503) from exc

    def state(self) -> dict:
        return self._call("/api/state")

    def go_to_base(self, x: float, y: float, height_cm: float) -> dict:
        return self._call("/api/go_to_base", {"x": x, "y": y, "height_cm": height_cm})

    def stop(self) -> dict:
        return self._call("/api/stop", {})


class SimError(RuntimeError):
    def __init__(self, message: str, status: int = 502) -> None:
        super().__init__(message)
        self.status = status
