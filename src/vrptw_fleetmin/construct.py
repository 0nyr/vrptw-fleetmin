"""Python facade over the private ``_native`` C++ extension.

This module re-exports the constructive heuristics and capacity lower bound
implemented in C++ (``src/cpp/``). End users should import from this module,
not from ``vrptw_fleetmin._native`` directly — the underscore-prefixed
extension is treated as a private implementation detail.

Available heuristics
--------------------
- :func:`nearest_neighbor` — classic greedy NN over multiple routes.
- :func:`regret_k_route_construction_best_insertion_travel_time` — regret-k
  best-insertion construction.
- :func:`parallel_route_construction_best_insertion_travel_time` — parallel
  best-insertion across routes.
- :func:`sequential_route_construction_nearest_neighbor_travel_time_wait` —
  Solomon-style sequential NN with time-window-aware wait cost.
- :func:`k_regret_insertion` — k-regret insertion variant.

Capacity bound
--------------
- :func:`capacity_lower_bound` — sum-demand / capacity LB on the vehicle count.
"""

from __future__ import annotations

from vrptw_fleetmin._native import (  # type: ignore[import-not-found]
    ConstructionResult,
    ConstructionStatus,
    VRPTWData,
    capacity_lower_bound,
    k_regret_insertion,
    nearest_neighbor,
    parallel_route_construction_best_insertion_travel_time,
    regret_k_route_construction_best_insertion_travel_time,
    sequential_route_construction_nearest_neighbor_travel_time_wait,
)

__all__ = [
    "ConstructionResult",
    "ConstructionStatus",
    "VRPTWData",
    "capacity_lower_bound",
    "k_regret_insertion",
    "nearest_neighbor",
    "parallel_route_construction_best_insertion_travel_time",
    "regret_k_route_construction_best_insertion_travel_time",
    "sequential_route_construction_nearest_neighbor_travel_time_wait",
]
