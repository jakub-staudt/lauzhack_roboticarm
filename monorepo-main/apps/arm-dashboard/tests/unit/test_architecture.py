"""The tier rules, enforced: who may import whom inside arm_dashboard, and where heavy libraries live.

Tier 1 ``core`` → tier 2 capabilities (``controller``, ``cameras``, ``viewer``, ``scan``) →
tier 4 delivery (``api``, ``main``). Imports flow downward only; capabilities agree through
``core.typing`` instead of importing each other; nothing imports the delivery tier.
"""

import ast
from collections.abc import Iterator
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[2] / 'src' / 'arm_dashboard'
PACKAGE = 'arm_dashboard'
CAPABILITIES = ('controller', 'cameras', 'viewer', 'scan')
DELIVERY = ('api', 'main')
# The collector subprocess: a separate process with its own entrypoint.
COLLECTOR = 'scan.worker'

# Third-party libraries that must stay inside the packages that need them.
CONFINED_LIBRARIES: dict[str, tuple[str, ...]] = {
    'cv2': (COLLECTOR,),
    'numpy': (COLLECTOR,),
    'fastapi': DELIVERY,
    'starlette': DELIVERY,
    'uvicorn': DELIVERY,
    'websockets': ('viewer', 'api'),
    'httpx': ('controller', 'viewer', COLLECTOR, 'api'),
}


def module_name(path: Path) -> str:
    """``src/arm_dashboard/scan/worker/mosaic.py`` → ``scan.worker.mosaic`` (package-relative)."""
    parts = path.relative_to(PACKAGE_ROOT).with_suffix('').parts
    return '.'.join(part for part in parts if part != '__init__')


def imports_of(path: Path) -> Iterator[str]:
    """Absolute module names imported by a file; relative imports resolved against it."""
    tree = ast.parse(path.read_text(), filename=str(path))
    package = module_name(path) if path.name == '__init__.py' else module_name(path).rpartition('.')[0]
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                base = package.split('.')[: len(package.split('.')) - node.level + 1] if package else []
                yield '.'.join([PACKAGE, *base, *(node.module or '').split('.')]).rstrip('.')
            elif node.module:
                yield node.module


def internal(name: str) -> str | None:
    """``arm_dashboard.core.typing`` → ``core.typing``; ``None`` for anything outside the package."""
    if name == PACKAGE or not name.startswith(PACKAGE + '.'):
        return None
    return name[len(PACKAGE) + 1 :]


def within(module: str, area: str) -> bool:
    return module == area or module.startswith(area + '.')


def tier_of(module: str) -> str:
    if within(module, 'core'):
        return 'core'
    if within(module, COLLECTOR):
        return COLLECTOR
    for capability in CAPABILITIES:
        if within(module, capability):
            return capability
    for delivery in DELIVERY:
        if within(module, delivery):
            return delivery
    return module


ALLOWED: dict[str, tuple[str, ...]] = {
    'core': ('core',),
    'controller': ('core', 'controller'),
    'cameras': ('core', 'cameras'),
    'viewer': ('core', 'viewer'),
    # The HTTP side of scanning never touches the collector package (it would drag in OpenCV).
    'scan': ('core', 'scan'),
    COLLECTOR: ('core', COLLECTOR),
    'api': ('core', *CAPABILITIES, 'api'),
    'main': ('core', 'api'),
}

SOURCES = sorted(PACKAGE_ROOT.rglob('*.py'))


def edges() -> Iterator[tuple[str, str, str]]:
    for path in SOURCES:
        source = module_name(path) or PACKAGE
        for name in imports_of(path):
            yield source, name, str(path.relative_to(PACKAGE_ROOT))


def test_sources_were_found() -> None:
    assert len(SOURCES) > 30


@pytest.mark.parametrize('area', list(ALLOWED))
def test_imports_flow_downward_only(area: str) -> None:
    violations = []
    for source, name, path in edges():
        target = internal(name)
        if target is None or tier_of(source) != area:
            continue
        target_tier = tier_of(target)
        allowed = ALLOWED[area]
        if target_tier not in allowed:
            violations.append(f'{path}: {source} imports {target}')
    assert violations == [], '\n'.join(violations)


def test_collector_does_not_import_the_scan_package_root() -> None:
    # `arm_dashboard.scan` re-exports the HTTP-side services; the collector needs none of them.
    offenders = [
        f'{path}: imports arm_dashboard.scan'
        for source, name, path in edges()
        if tier_of(source) == COLLECTOR and internal(name) == 'scan'
    ]
    assert offenders == []


@pytest.mark.parametrize('library', list(CONFINED_LIBRARIES))
def test_libraries_stay_where_they_belong(library: str) -> None:
    homes = CONFINED_LIBRARIES[library]
    offenders = [
        f'{path}: {source} imports {name}'
        for source, name, path in edges()
        if (name == library or name.startswith(library + '.'))
        and not any(within(source, home) for home in homes)
    ]
    assert offenders == [], '\n'.join(offenders)


def test_only_settings_reads_the_environment() -> None:
    readers = [
        str(path.relative_to(PACKAGE_ROOT))
        for path in SOURCES
        if 'os.environ' in path.read_text() or 'os.getenv' in path.read_text()
    ]
    assert readers == []  # pydantic-settings reads it; no module touches os.environ directly
