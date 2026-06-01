"""End-to-end test for the ILS dispatcher.

Covers:

- ``ils_vehicle_min(..., variant='v4')`` on Solomon C101 (n=100) reaches the
  BKS fleet size (10) within a short time budget.
- The dispatcher validates the ``variant`` argument.
- Each variant is at least callable with a tiny time budget (smoke).
"""

from __future__ import annotations

import pytest

from vrptw_fleetmin import ils_vehicle_min
from vrptw_fleetmin.ils import ILSResult, ILSTrace

from _helpers import TEST_SCALING_FACTOR, scale_instance_for_ils


def test_dispatcher_rejects_unknown_variant(c101_instance_and_bks):
    instance, _bks = c101_instance_and_bks
    scaled = scale_instance_for_ils(instance)
    with pytest.raises(ValueError, match="Unknown ILS variant"):
        ils_vehicle_min(scaled, time_limit_s=1.0, seed=42, variant="v99")


@pytest.mark.parametrize("variant", ["v1", "v2", "v3", "v4"])
def test_each_variant_is_callable(c101_instance_and_bks, variant):
    """Smoke: every variant runs end-to-end on C101 with a 2 s budget and
    returns a well-formed :class:`ILSResult`."""
    instance, _bks = c101_instance_and_bks
    scaled = scale_instance_for_ils(instance)
    result = ils_vehicle_min(
        scaled,
        time_limit_s=2.0,
        seed=42,
        variant=variant,
        scaling_factor=TEST_SCALING_FACTOR,
    )
    assert isinstance(result, ILSResult)
    assert isinstance(result.trace, ILSTrace)
    assert result.num_routes == len(result.routes) > 0
    assert result.wall_time > 0
    # Every customer 1..100 appears exactly once.
    visited = [c for route in result.routes for c in route]
    assert sorted(visited) == list(range(1, instance.num_customers + 1))


@pytest.mark.slow
def test_v4_reaches_bks_fleet_on_c101(c101_instance_and_bks):
    """Headline test for the ILS: with a 10 s budget on C101 (BKS fleet = 10),
    ``variant='v4'`` should match the BKS fleet count."""
    instance, bks = c101_instance_and_bks
    scaled = scale_instance_for_ils(instance)
    result = ils_vehicle_min(
        scaled,
        time_limit_s=10.0,
        seed=42,
        variant="v4",
        scaling_factor=TEST_SCALING_FACTOR,
    )
    assert result.num_routes == bks.num_routes == 10, (
        f"v4 fleet {result.num_routes} != BKS fleet {bks.num_routes} on C101"
    )
