# Running the arm agent on the RAMION server (needs the user's explicit approval for each step)

What: `arm_agent.py` (+ `realarm.py`, the arm's click-move code) runs on the server next to the controller and drives it through
`http://100.85.56.14:8780` (this machine's Tailscale address, where the dashboard listens; no network hop). The laptop sends one `/go` command and `/stop`, and polls `/state`. Tested only against a fake controller.

Proposed, minimal footprint (nothing touches /srv/ramion, /etc/ramion, /var/lib/ramion, ramion-* services, sshd, tailscaled, USB):
1. Read-only look first: which Python has `soarm_lab` importable (the release venv), free disk, who owns what. No changes.
2. Copy the bundle (`tools/make_agent_bundle.sh`, 20 KB) into the SSH user's home: `~/arm_agent/`. No sudo.
3. Make a token (`head -c 32 /dev/urandom | base64 > ~/arm_agent/token`, mode 600) and copy it to the laptop.
4. Start it by hand in a `tmux`/`nohup` session, NOT at boot, listening on the Tailscale IP only:
   `PYTHONPATH=~/arm_agent <release-venv>/bin/python -m lauzkach_eye_robot_control.arm_agent --config ~/arm_agent/agent_config.json
   --token-file ~/arm_agent/token --host 100.85.56.14 --port 8790`
5. On the laptop: `server.py ... --real-agent http://100.85.56.14:8790 --agent-token-file token`.

Blast radius: one extra user-level process and one extra port (8790, token-protected, Tailscale-only). It can only do what the
dashboard API already allows (claim control, send joint targets, soft stop). It never latches the e-stop or changes torque/speed.
Undo: stop the process (Ctrl-C / `kill <pid>`) and `rm -rf ~/arm_agent`. The manual page and the RAMION services are unaffected;
the laptop can fall back to `--real-arm` (local driving) at any time.
Safety kept: hover >= 3 cm, joint-limit/gripper/e-stop/other-owner checks, control claimed after planning, soft stop, agent dead-man
(arm held if nobody polls `/state` for 4 s during a move), SIGTERM stops the arm and exits.

## Done so far (2026-11-22, with the user's approval)
Read-only look: server has tmux, Python 3.12 without pip/venv, 278 GB free, ports 8765/8780 on the Tailscale IP, RAMION venvs unreadable.
Installed in the SSH user's home only: `~/.local/bin/uv` 0.13.9 (checksum verified) and `~/arm_agent/venv` (numpy, mujoco). Still to
do: `uv pip install pyyaml`, copy the bundle (6.6 MB, includes minimal `soarm`/`soarm_lab` copies from monorepo-main), start by hand.
