"""Pick a block and put it somewhere else: the whole grasp sequence, step after step, without button presses.

It only calls the arm agent's staged actions in order (the same ones the old step buttons called) and waits for each to finish:
open the gripper, hover over the block, lower (the gripper camera can stop it), grip, lift, carry to the destination, place (lower,
release, rise clear, park). If any step fails or Stop is pressed, it stops at once and says which step and why; the arm holds where it is.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

STEPS = (("open", "Opening the gripper"), ("hover", "Moving over the block"), ("lower", "Lowering onto the block"), ("close", "Gripping the block"),
         ("lift", "Lifting the block"), ("carry", "Carrying the block to the destination"), ("place", "Placing the block and parking"))


class PickPlaceRunner:
    def __init__(self, step: Callable[[str], dict], task: Callable[[], dict], note: Callable[[str], None] = lambda s: None,
                 step_timeout_s: float = 150.0, poll_s: float = 0.4) -> None:
        """`step(action)` starts one agent action (raises RuntimeError with a plain reason if refused); `task()` returns the agent's state:
        {"task": {"state", "message"}, "grasp": {"stage"}}."""
        self._step, self._task, self._note = step, task, note
        self.timeout_s, self.poll_s = step_timeout_s, poll_s
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None
        self.status: dict = {"state": "idle", "step": 0, "total": len(STEPS), "name": "", "message": ""}

    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, label: str, first: int = 0) -> None:
        """`first`: index of the first step to run (the earlier ones were already done: the arm is already holding the block)."""
        if self.busy():
            raise RuntimeError("A pick and place is already running.")
        self._cancel.clear()
        self._first = first
        self.status = {"state": "running", "step": first, "total": len(STEPS), "name": "", "message": "", "label": label}
        self._thread = threading.Thread(target=self._run, daemon=True, name="pickplace")
        self._thread.start()

    def cancel(self) -> None:
        self._cancel.set()

    def _set(self, **kw) -> None:
        self.status = {**self.status, **kw}

    def _wait(self) -> dict:
        end = time.monotonic() + self.timeout_s
        while time.monotonic() < end:
            if self._cancel.is_set():
                return {"state": "stopped", "message": "Stopped."}
            try:
                st = self._task()
            except Exception as exc:                                      # noqa: BLE001 - a missed poll is not a failure
                time.sleep(self.poll_s)
                continue
            if st["task"]["state"] not in ("running", "planning", "moving"):
                return {"state": st["task"]["state"], "message": st["task"].get("message", ""), "stage": (st.get("grasp") or {}).get("stage")}
            time.sleep(self.poll_s)
        return {"state": "error", "message": "The step took too long."}

    def _run(self) -> None:
        for i, (action, name) in enumerate(STEPS, start=1):
            if i - 1 < getattr(self, "_first", 0):
                continue
            if self._cancel.is_set():
                self._set(state="stopped", message="Stopped before step %d (%s)." % (i, name.lower()))
                return
            self._set(step=i, name=name, message="")
            try:
                self._step(action)
            except RuntimeError as exc:
                self._fail(i, name, str(exc))
                return
            result = self._wait()
            if self._cancel.is_set() or result["state"] == "stopped":
                self._set(state="stopped", message=f"Stopped during step {i} ({name.lower()}). The arm holds where it is.")
                return
            if result["state"] != "done":
                self._fail(i, name, result["message"])
                return
            if action == "close" and result.get("stage") != "closed":
                self._fail(i, name, "The jaws did not end up holding a block.")
                return
            if result.get("message"):
                self._note(f"Step {i} of {len(STEPS)}: {result['message']}")
        self._set(state="done", step=len(STEPS), name="Done", message="The block is at the destination and the arm is parked.")

    def _fail(self, i: int, name: str, why: str) -> None:
        self._set(state="error", message=f"Step {i} of {len(STEPS)} ({name.lower()}) stopped: {why} The arm holds where it is.")
        self._note(self.status["message"])
