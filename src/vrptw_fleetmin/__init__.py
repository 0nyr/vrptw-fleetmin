"""vrptw-fleetmin: fast VRPTW constructive heuristics and ILS-based fleet-size minimizer.

The ILS uses the local-search and repair operators of PyVRP 0.12.2. It is a
separate algorithm from the ``IteratedLocalSearch`` solver shipped by PyVRP
0.13 and later.

Public API
----------
- :func:`vrptw_fleetmin.ils_vehicle_min` — single ILS entry point with a
  ``variant=`` parameter selecting one of four variants (default
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
