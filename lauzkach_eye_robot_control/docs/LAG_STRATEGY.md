# Working over a slow link: compute near the arm, send intents

## The problem, with numbers
The arm is in another country, reached over a relayed Tailscale path. Measured: round trips of 150-500 ms with spikes past 2 s,
and no direct connection. The arm's controller makes this worse: it refuses velocity input whose robot feedback is older than
0.75 s, holds a motion for at most 0.35 s without renewal, and drops the control lease after 1.5 s without a heartbeat. So
anything time-critical (heartbeats, key-hold motion, streaming targets) fails whenever the link stalls. **Rule: nothing
time-critical may cross the link.**

## Principle
The browser sends small **intents** ("go to this click", "next corner", "stop"). Everything else runs next to the arm:

| Where | What |
| --- | --- |
| Browser | draw the picture and overlays, capture clicks, show state and link quality. Nothing time-critical. |
| Server next to the arm | camera detection and the 5 s rectangle lock, picture -> arm coordinates, IK and motion planning, trajectory timing and execution against the controller on localhost, safety checks (reach, joint limits, floor), soft stop, dead-man timer, log. |

Detection and calibration on the server also mean the lock survives page reloads and the picture can be low-rate (2 fps): the
browser never needs fast video for the arm to behave.

## The command protocol (built and tested, in `drive.py` and `service.py`)
Every move command carries `{cid, sent_ms, gen, max_age_ms}`:

* **Stale commands are refused.** `sent_ms` is on the server's clock (the page estimates the offset from its quickest round
  trip). A click that arrives more than 2 s late is not executed: a lag spike can never make the arm move "later, to where you
  clicked a while ago".
* **Retries are safe.** A lost reply is retried with the same `cid`; the server remembers recent ids and returns the first
  answer instead of moving again.
* **Stop voids what is in flight.** Stop increments a *generation*. A move sent before the Stop but arriving after it carries
  the old generation and is refused. A Stop itself is always accepted, whatever its age.
* **Dead-man.** While a move runs, any request from the page counts as "still there". If the page is silent for 3 s the arm is
  held where it is and the generation is bumped.
* **One move at a time**, with the move buttons disabled while the arm is busy, and the link round trip shown on the page.

Stop is a **soft stop**: it halts the arm and cancels pending moves, but latches nothing, so the next move can be sent at once.
The controller's emergency stop stays a separate, clearly labelled button.

## What the real arm still needs (not built)
1. **Run this next to the arm.** Plan and execute on the server; the controller is then 1 ms away, so heartbeats and
   streaming targets no longer depend on the link. The existing client-side Mover (`robotic_arm/clickmove.py`) already sends
   one joint target at a time and waits until it is reached, which tolerates lag, but it still needs a heartbeat every 250 ms.
2. **Plan preview and confirm.** The server computes the path, returns it for the page to draw, and a move starts only on a
   short-lived confirmation bound to that exact plan (so a delayed double-click cannot confirm a different plan).
3. **Soft stop = the controller's `stop`** (holds the pose and releases the lease), then "Take control" again; never the
   emergency-stop latch unless a person is at the arm.
4. **Server-side limits** on speed, reach, floor and joint jumps, independent of what the page asks for.

## Failure behaviour
| What goes wrong | What happens |
| --- | --- |
| Link stalls, then delivers an old click | refused as stale; nothing moves |
| Reply lost, page retries | same `cid`: answered from memory, no second move |
| User presses Stop while a move is queued in the link | the queued move arrives with the old generation and is refused |
| Page closed or link lost during a move | dead-man holds the arm after 3 s |
| Page server crashes | the controller's own watchdogs hold the arm (0.35 s motion, 1.5 s lease) |
| Simulator / arm unreachable | the page says so; Stop reports that it could not confirm |

## Putting it on the server (needs explicit approval first)
Own folder and unprivileged user; not started at boot; no changes under `/srv/ramion`, `/etc/ramion` or any `ramion-*` service;
CPU and memory limits; one-line undo. Order: a read-only client first; then the simulator with real camera clicks (today);
then a supervised hover-only move on the real arm with someone at it.
