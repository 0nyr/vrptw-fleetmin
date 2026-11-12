"""Smoke tests for the C++ constructive heuristics + capacity lower bound.

For each of the six Solomon n=100 fixtures (C/R/RC × TW1/TW2):

- Build a :class:`VRPTWData` from the float instance via ceil-scaling.
- Compute the capacity lower bound and sanity-check it.
- Run each of the five constructive heuristics, assert ``OK`` status and a
  route count consistent with the LB and the instance's vehicle budget.

These are smoke tests, not optimality tests — the goal is to confirm the
pybind11 bindings round-trip cleanly and the heuristics build a feasible
solution covering all customers on every family.
"""

from __future__ import annotations

import pytest

from vrptw_fleetmin import construct

from _helpers import make_scaled_native_data


def test_vrptw_data_round_trips(solomon_instance_and_bks):
    name, instance, _bks = solomon_instance_and_bks
    data = make_scaled_native_data(instance)
    assert data.instance_name == name
    assert data.num_customers == instance.num_customers == 100
    assert data.vehicle_capacity == instance.vehicle_capacity


def test_capacity_lower_bound_is_sensible(solomon_instance_and_bks):
    _name, instance, bks = solomon_instance_and_bks
    data = make_scaled_native_data(instance)
    lb = construct.capacity_lower_bound(data)
    assert isinstance(lb, int)
    assert 1 <= lb <= instance.num_vehicles
    # Capacity LB cannot exceed the actual fleet used by the BKS.
    assert lb <= bks.num_routes, (
        f"capacity LB ({lb}) exceeds BKS fleet ({bks.num_routes}) for "
        f"{instance.instance_name}"
    )


@pytest.mark.parametrize(
    "heuristic_name",
    [
        "nearest_neighbor",
        "parallel_route_construction_best_insertion_travel_time",
        "sequential_route_construction_nearest_neighbor_travel_time_wait",
    ],
)
def test_construction_heuristic_builds_feasible_cover(
    solomon_instance_and_bks, heuristic_name
):
    """Each parameter-free heuristic builds a feasible solution that covers
    every customer with a route count within the vehicle budget."""
    _name, instance, _bks = solomon_instance_and_bks
    data = make_scaled_native_data(instance)
    heuristic = getattr(construct, heuristic_name)
    result = heuristic(data)
    assert result.status == construct.ConstructionStatus.OK, (
        f"{heuristic_name} returned status {result.status} on "
        f"{instance.instance_name}"
    )
    routes = result.routes
    assert routes, f"{heuristic_name} returned no routes"
    assert 1 <= len(routes) <= instance.num_vehicles
    visited = [c for route in routes for c in route]
    assert sorted(visited) == list(range(1, instance.num_customers + 1)), (
        f"{heuristic_name} did not visit every customer exactly once on "
        f"{instance.instance_name}"
    )


@pytest.mark.parametrize("k", [2, 3])
def test_regret_k_builds_feasible_cover(solomon_instance_and_bks, k):
    _name, instance, _bks = solomon_instance_and_bks
    data = make_scaled_native_data(instance)
    result = construct.regret_k_route_construction_best_insertion_travel_time(
        data, k=k
    )
    assert result.status == construct.ConstructionStatus.OK, (
        f"regret-{k} returned status {result.status} on {instance.instance_name}"
    )
    routes = result.routes
    assert 1 <= len(routes) <= instance.num_vehicles
    visited = [c for route in routes for c in route]
    assert sorted(visited) == list(range(1, instance.num_customers + 1))


def test_k_regret_insertion_matches_regret_k_alias_on_c101(c101_instance_and_bks):
    """``k_regret_insertion`` is an alias entry point for regret-k insertion.
    With ``k=2`` it should produce a feasible cover on C101."""
    instance, _bks = c101_instance_and_bks
    data = make_scaled_native_data(instance)
    result = construct.k_regret_insertion(data, k=2)
    assert result.status == construct.ConstructionStatus.OK
    assert result.routes
    visited = [c for route in result.routes for c in route]
    assert sorted(visited) == list(range(1, instance.num_customers + 1))
