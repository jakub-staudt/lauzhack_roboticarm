# syntax=docker/dockerfile:1
# Arm dashboard image: the FastAPI app and its OpenCV scan collector, nothing else.
# Build context: apps/arm-dashboard. See deploy/compose.yaml at the repository root.

ARG PYTHON_IMAGE=python:3.12-slim-bookworm
ARG UV_IMAGE=ghcr.io/astral-sh/uv:0.13.7

FROM ${UV_IMAGE} AS uv

# --- build: resolve exactly uv.lock into /app/.venv -------------------------------------------
FROM ${PYTHON_IMAGE} AS build
COPY --from=uv /uv /usr/local/bin/uv
ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never \
    UV_PROJECT_ENVIRONMENT=/app/.venv
WORKDIR /src
# Dependencies first, so source edits do not invalidate the (large, OpenCV) dependency layer.
COPY pyproject.toml uv.lock .python-version ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-dev --extra scan --no-install-project
COPY src ./src
# --no-editable: the package (and its static pages) is copied into the venv; /src is not needed at runtime.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-dev --extra scan --no-editable

# --- runtime ----------------------------------------------------------------------------------
FROM ${PYTHON_IMAGE} AS runtime
# opencv-contrib-python (not headless) needs these shared libraries at import time.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
RUN useradd --system --uid 10001 --user-group --home-dir /app --no-create-home dashboard \
    && install -d -o dashboard -g dashboard /var/lib/ramion/scans
COPY --from=build /app/.venv /app/.venv
# Inside the container the app listens on all interfaces; compose publishes it on one host IP.
ENV PATH=/app/.venv/bin:$PATH \
    PYTHONUNBUFFERED=1 \
    RAMION_DASHBOARD_SERVER__HOST=0.0.0.0 \
    RAMION_DASHBOARD_SERVER__PORT=8780 \
    RAMION_DASHBOARD_SCAN__STORAGE_DIR=/var/lib/ramion/scans
WORKDIR /app
USER dashboard
EXPOSE 8780
# /healthz answers without the controller, so a down arm controller never restarts the dashboard.
HEALTHCHECK --interval=15s --timeout=3s --start-period=10s --retries=3 \
    CMD ["python", "-c", "import sys, urllib.request; sys.exit(urllib.request.urlopen('http://127.0.0.1:8780/healthz', timeout=2).status != 200)"]
# uvicorn finishes open requests on SIGTERM and the lifespan stops a running scan collector.
STOPSIGNAL SIGTERM
ENTRYPOINT ["arm-dashboard"]
