"""vrptw-fleetmin: fast VRPTW constructive heuristics and ILS-based fleet-size minimizer.

Local-search backend: PyVRP 0.12.2 HGS (Hybrid Genetic Search). This package does
NOT use the ILS introduced in PyVRP 0.13+. The "ILS" exposed here is a separate
algorithm developed in the context of paper6.
"""

from __future__ import annotations

# Public API placeholders; populated once the ILS module is extracted.
# from vrptw_fleetmin.ils import ils_vehicle_min  # noqa: F401
# from vrptw_fleetmin import construct  # noqa: F401

__version__ = "0.0.1"
