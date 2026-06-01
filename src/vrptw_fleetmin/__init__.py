"""vrptw-fleetmin: fast VRPTW constructive heuristics and ILS-based fleet-size minimizer.

Local-search backend: PyVRP 0.12.2 HGS (Hybrid Genetic Search). This package does
NOT use the ILS introduced in PyVRP 0.13+. The "ILS" exposed here is a separate
algorithm developed in the context of paper6.

Public API
----------
- :func:`vrptw_fleetmin.ils_vehicle_min` — single ILS entry point with a
  ``variant=`` parameter selecting one of four published variants (default
  ``'v4'``).
- :mod:`vrptw_fleetmin.construct` — Python facade over the C++ constructive
  heuristics (NN, BI, RG2, RG3) and capacity lower bound.
"""

from __future__ import annotations

from vrptw_fleetmin.ils import ILSResult, ILSTrace, ils_vehicle_min
from vrptw_fleetmin import construct

__all__ = [
    "ILSResult",
    "ILSTrace",
    "construct",
    "ils_vehicle_min",
]

__version__ = "0.1.0"
