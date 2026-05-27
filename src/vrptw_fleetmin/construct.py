"""Python facade over the private `_native` C++ extension.

This module re-exports the constructive heuristics and capacity lower bound
implemented in C++ (`src/cpp/`) with proper type hints. End users should import
from this module, not from `_native` directly.

TODO: extraction step 2 — populate re-exports once the C++ binding has been
renamed from `paper6_native` to `_native`.
"""

from __future__ import annotations

# from vrptw_fleetmin._native import (  # type: ignore[import-not-found]
#     VRPTWData,
#     ConstructionResult,
#     ConstructionStatus,
#     capacity_lower_bound,
#     nearest_neighbor,
#     regret_k_route_construction_best_insertion_travel_time,
#     parallel_route_construction_best_insertion_travel_time,
#     sequential_route_construction_nearest_neighbor_travel_time_wait,
#     k_regret_insertion,
# )

__all__: list[str] = []
