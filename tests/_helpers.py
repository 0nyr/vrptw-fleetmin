"""Shared helpers for the vrptw-fleetmin test suite.

Not a conftest because pytest treats ``conftest.py`` specially (fixtures
auto-loaded into tests). Plain importable code lives here.
"""

from __future__ import annotations

from math import ceil
from pathlib import Path

from mamut_routing_lib.models import BenchmarkInstance

from vrptw_fleetmin.construct import VRPTWData


FIXTURES_DIR = Path(__file__).parent / "fixtures" / "sintef2008_n100"

INSTANCE_NAMES: list[str] = ["C101", "C201", "R101", "R201", "RC101", "RC201"]

TEST_SCALING_FACTOR = 1000
"""Scale Solomon float arc costs / windows / service times by this factor
before rounding to int, matching paper6's default ``P=1000``."""


def make_scaled_native_data(
    instance: BenchmarkInstance, scaling_factor: int = TEST_SCALING_FACTOR
) -> VRPTWData:
    """Build a :class:`VRPTWData` from a float :class:`BenchmarkInstance`.

    Solomon instances carry float arc costs / time windows / service times; the
    C++ heuristics work in integers. This applies a uniform ceil-scaled cast,
    matching the scaling pattern paper6 uses for the same heuristics.
    """
    arc_costs_scaled: list[list[int]] = [
        [int(ceil(c * scaling_factor)) for c in row] for row in instance.arc_costs
    ]
    time_windows_scaled: list[tuple[int, int]] = [
        (int(ceil(tw[0] * scaling_factor)), int(ceil(tw[1] * scaling_factor)))
        for tw in instance.time_windows
    ]
    service_times_scaled: list[int] = [
        int(ceil(s * scaling_factor)) for s in instance.service_times
    ]
    coordinates: list[tuple[float, float]] = [
        (c[0], c[1]) for c in instance.coordinates
    ]

    return VRPTWData(
        instance_name=instance.instance_name,
        num_customers=instance.num_customers,
        num_vehicles=instance.num_vehicles,
        vehicle_capacity=instance.vehicle_capacity,
        coordinates=coordinates,
        demands=instance.demands,
        service_times=service_times_scaled,
        time_windows=time_windows_scaled,
        arc_travel_times=arc_costs_scaled,
    )


def scale_instance_for_ils(
    instance: BenchmarkInstance, scaling_factor: int = TEST_SCALING_FACTOR
) -> BenchmarkInstance:
    """Return a model-copy of ``instance`` with arc costs / windows / service
    times ceil-scaled by ``scaling_factor``. The ILS Python entry point wants
    a :class:`BenchmarkInstance`, not a :class:`VRPTWData`."""
    arc_costs_scaled = [
        [int(ceil(c * scaling_factor)) for c in row] for row in instance.arc_costs
    ]
    time_windows_scaled = [
        (int(ceil(tw[0] * scaling_factor)), int(ceil(tw[1] * scaling_factor)))
        for tw in instance.time_windows
    ]
    service_times_scaled = [
        int(ceil(s * scaling_factor)) for s in instance.service_times
    ]
    return instance.model_copy(
        update={
            "arc_costs": arc_costs_scaled,
            "time_windows": time_windows_scaled,
            "service_times": service_times_scaled,
        }
    )
