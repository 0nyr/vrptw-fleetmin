"""Pytest fixtures for the vrptw-fleetmin test suite.

The benchmark fixtures live in ``tests/fixtures/sintef2008_n100/`` and consist
of one Solomon instance per family (C/R/RC) × time-window class (1/2) at n=100.
They are loaded via ``mamut_routing_lib.artifacts.load_benchmark_instance`` and
``load_bks``. The fixture directory also contains the LICENSE / README / NOTICE
files required by the MAMUT-routing redistribution clause.

Plain helpers (scaling, constants) live in ``_helpers.py`` so they can be
imported normally; conftest.py is reserved for pytest fixtures.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from mamut_routing_lib.artifacts import load_benchmark_instance, load_bks
from mamut_routing_lib.models import BenchmarkBKS, BenchmarkInstance

from _helpers import FIXTURES_DIR, INSTANCE_NAMES


@pytest.fixture(scope="session")
def fixtures_dir() -> Path:
    return FIXTURES_DIR


@pytest.fixture(scope="session", params=INSTANCE_NAMES)
def solomon_instance_and_bks(
    request: pytest.FixtureRequest,
) -> tuple[str, BenchmarkInstance, BenchmarkBKS]:
    """Parametrize each test over the 6-instance fixture set."""
    name: str = request.param
    instance = load_benchmark_instance(FIXTURES_DIR / f"{name}.vrp.json")
    bks = load_bks(FIXTURES_DIR / f"{name}.bks.HierarchicalVehicleCost.json")
    return name, instance, bks


@pytest.fixture(scope="session")
def c101_instance_and_bks() -> tuple[BenchmarkInstance, BenchmarkBKS]:
    """C101 only — used by the headline ILS test that targets the BKS fleet."""
    instance = load_benchmark_instance(FIXTURES_DIR / "C101.vrp.json")
    bks = load_bks(FIXTURES_DIR / "C101.bks.HierarchicalVehicleCost.json")
    return instance, bks
