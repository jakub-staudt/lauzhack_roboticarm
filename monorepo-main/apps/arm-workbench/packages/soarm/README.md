# soarm

Hardware control and the robot model for the SO-101 arm. The package provides Feetech STS3215 and LeRobot backends, the arm controller API, and a serial-to-TCP bridge. It also registers the robot and backends as plugins for the [Machinekind digital twin](https://github.com/machinekind/twin).

## Get started

From the repository root, follow the [setup instructions](../../README.md#get-started) and [safety rules](../../docs/safety.md) before powering the arm.

```powershell
uv sync
uv run soarm-panel
```

Start the [dashboard](../../../arm-dashboard/README.md#develop) for browser operation. Its panel provides manual joint/gripper controls, measured and requested positions, home, STOP, ownership, torque settings, and diagnostics. It selects a single USB-serial device automatically; specify `--port COM5` or `--port /dev/ttyACM0` when needed.

For cameras and the read-only measured-pose 3D viewer:

```powershell
uv sync --group workbench
uv run --group workbench soarm-panel --listen 0.0.0.0:8765 --overhead-camera 1 --wrist-camera 2 --no-browser
```

Camera indices `1` and `2` belong to the observed development laptop. Assign your own indices through CLI options or Session; assignments apply only to the running process. On Windows, stop the old panel before syncing to avoid a locked executable. Missing optional camera/rendering dependencies leave the base panel usable. The Model tab has no policy runner configured. The viewer enlarges in place; phones and short landscape screens show one selected media view above compact manual controls.

The controller API defaults to localhost:8765. The dashboard UI defaults to localhost:8780. LAN camera streams use that same port; the 3D viewer normally uses a separate port 8766 (`--viewer-port`, or `--no-viewer` to disable). Scope firewall access to the application and intended trusted devices. Any reachable client can request STOP and compete for an available control lease; this is not user authentication.

Select **Take control** before moving. Only the owning browser can send motion commands; its heartbeat runs every 400ms and expires after 1.5s. Hiding/leaving the page releases control. STOP or Esc holds the measured pose and revokes ownership, preserving the existing torque-enable state. Ctrl-C exits while keeping torque on. Support the arm before disabling torque in Session > Advanced, and retain access to the physical power disconnect.

Targets are constrained by robot configuration and EEPROM limits. The 3D view displays measured positions; it does not verify collision clearance or physical safety. See [WORKBENCH.md](../../docs/WORKBENCH.md) for operation, camera troubleshooting, diagnostics and API contracts, and [first-contact.md](../../docs/first-contact.md) for assembly/calibration.

## Serial bridge

Run the bridge in one terminal, then the panel in another:

```powershell
uv run soarm-bridge --port COM5 --listen 127.0.0.1:5555
uv run soarm-panel --port socket://127.0.0.1:5555
```

For remote bridge access, bind to the host network address and restrict clients with `--allow <client-ip>`. The Feetech backend accepts `socket://host:port` directly.

## Integration and dependencies

The twin discovers robot `so101` and backends `feetech` and `lerobot` through entry points. The [robot folder](soarm/robot/README.md) contains its MuJoCo model, meshes, and `robot.yaml`.

The base dependency is `machinekind-twin-api`. Optional extras provide:

- `feetech`: serial communication and the bridge.
- `panel`: manual browser panel.
- `workbench`: panel plus OpenCV cameras and the twin rendering dependencies.
- `lerobot`: LeRobot backend, requiring Python 3.12+.

Default workspace `uv sync` includes `feetech` and `panel`; `--group workbench` adds cameras and rendering. `soarm-panel` remains the entry point. See [conventions](../../docs/conventions.md) for units and plugin contracts and [soarm-lab](../soarm_lab/README.md) for simulation/training.

## License

[Apache-2.0](LICENSE), with attribution in [NOTICE](NOTICE). The bundled robot model and meshes retain their upstream license and [third-party notices](THIRD_PARTY_NOTICES).
