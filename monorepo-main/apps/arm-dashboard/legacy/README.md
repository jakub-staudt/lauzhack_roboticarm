# Legacy dashboard (standard library)

The proxy that served port 8780 before the FastAPI app in `../src/arm_dashboard`, kept as it was
deployed. It is **not deployed** from this tree any more and nothing in the app imports it.

It stays for two reasons until the FastAPI dashboard has run in production for a while:

- **Reference.** The contract suite (`../tests/contract`) runs every test against this code and the
  FastAPI app; `../tests/unit/test_legacy_parity.py` and the collector tests compare results with it.
- **Rollback.** Releases built from it still exist on the server. `deploy/ramion-dashboard.service` and
  `scripts/deploy.sh` keep starting and checking them.

| File | Became |
| --- | --- |
| `server.py` | `arm_dashboard.api`, `controller`, `cameras` |
| `viewer_proxy.py` | `arm_dashboard.viewer` |
| `scan_manager.py` | `arm_dashboard.scan` |
| `scan_worker.py` | `arm_dashboard.scan.worker` |
| `requirements-scan.txt` | the `scan` extra in `../pyproject.toml` |
| `index.html` | nothing; it was no longer served |
| `source-snapshot.json` | historical record of the 2026-11-03 import from the server |

The map/scan pages it serves now live in `../src/arm_dashboard/static/`; `server.py` reads them there.
Its own tests (OpenCV needed for the collector ones):

```bash
cd apps/arm-dashboard/legacy
python3 -m unittest discover -s tests -v
```

This directory is deleted once the FastAPI dashboard has run in production long enough that no rollback to a legacy release is needed.
